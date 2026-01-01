/**
 * @file SerialPort.cpp
 * @brief 现代C++跨平台串口通信库实现
 * @author CSerialPort Team
 * @version 3.0.0
 * @date 2026-01-01
 * 
 * @copyright MIT License
 */

#include "SerialPort.h"
#include <algorithm>
#include <cstring>
#include <sstream>

#if CSERIALPORT_PLATFORM_WINDOWS
    #include <setupapi.h>
    #pragma comment(lib, "setupapi.lib")
#elif CSERIALPORT_PLATFORM_LINUX
    #include <sys/stat.h>
    #include <glob.h>
#endif

namespace csp {

// ============================================================================
// 平台相关实现类
// ============================================================================

class SerialPort::Impl {
public:
    Impl() = default;
    ~Impl() { close(); }
    
    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;
    
    // ========================================================================
    // 打开/关闭
    // ========================================================================
    
    VoidResult open(const std::string& portName, const SerialConfig& config) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (isOpen_) {
            return VoidResult(ErrorCode::AlreadyOpen, "Port is already open");
        }
        
        portName_ = portName;
        config_ = config;
        
#if CSERIALPORT_PLATFORM_WINDOWS
        std::string devicePath = "\\\\.\\" + portName;
        
        handle_ = CreateFileA(
            devicePath.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,
            nullptr,
            OPEN_EXISTING,
            FILE_FLAG_OVERLAPPED,
            nullptr
        );
        
        if (handle_ == INVALID_HANDLE_VALUE) {
            DWORD error = GetLastError();
            if (error == ERROR_FILE_NOT_FOUND) {
                return VoidResult(ErrorCode::PortNotFound, "Port not found: " + portName);
            } else if (error == ERROR_ACCESS_DENIED) {
                return VoidResult(ErrorCode::PermissionDenied, "Access denied: " + portName);
            } else if (error == ERROR_SHARING_VIOLATION) {
                return VoidResult(ErrorCode::PortBusy, "Port is busy: " + portName);
            }
            return VoidResult(ErrorCode::OpenFailed, "Failed to open port: " + portName);
        }
        
        readEvent_ = CreateEvent(nullptr, TRUE, FALSE, nullptr);
        writeEvent_ = CreateEvent(nullptr, TRUE, FALSE, nullptr);
        shutdownEvent_ = CreateEvent(nullptr, TRUE, FALSE, nullptr);
        
        if (!readEvent_ || !writeEvent_ || !shutdownEvent_) {
            closeInternal();
            return VoidResult(ErrorCode::OpenFailed, "Failed to create events");
        }
        
        if (!SetupComm(handle_, static_cast<DWORD>(config.readBufferSize), 
                       static_cast<DWORD>(config.writeBufferSize))) {
            closeInternal();
            return VoidResult(ErrorCode::ConfigFailed, "Failed to setup comm buffers");
        }
        
#elif CSERIALPORT_PLATFORM_LINUX
        fd_ = ::open(portName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        
        if (fd_ < 0) {
            if (errno == ENOENT) {
                return VoidResult(ErrorCode::PortNotFound, "Port not found: " + portName);
            } else if (errno == EACCES) {
                return VoidResult(ErrorCode::PermissionDenied, "Access denied: " + portName);
            } else if (errno == EBUSY) {
                return VoidResult(ErrorCode::PortBusy, "Port is busy: " + portName);
            }
            return VoidResult(ErrorCode::OpenFailed, "Failed to open port: " + portName);
        }
        
        if (tcgetattr(fd_, &originalTermios_) != 0) {
            ::close(fd_);
            fd_ = -1;
            return VoidResult(ErrorCode::ConfigFailed, "Failed to get terminal attributes");
        }
#endif
        
        auto result = applyConfig(config);
        if (!result) {
            closeInternal();
            return result;
        }
        
        isOpen_ = true;
        statistics_.reset();
        
        return VoidResult();
    }
    
    VoidResult close() {
        std::lock_guard<std::mutex> lock(mutex_);
        stopAsyncReceiveInternal();
        closeInternal();
        return VoidResult();
    }
    
    bool isOpen() const noexcept {
        return isOpen_;
    }
    
    SerialConfig config() const noexcept {
        return config_;
    }
    
    VoidResult setConfig(const SerialConfig& config) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!isOpen_) {
            return VoidResult(ErrorCode::NotOpen, "Port is not open");
        }
        
        auto result = applyConfig(config);
        if (result) {
            config_ = config;
        }
        return result;
    }
    
    // ========================================================================
    // 同步读取
    // ========================================================================
    
    Result<ByteBuffer> read(size_t maxBytes, std::optional<Duration> timeout) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!isOpen_) {
            return Result<ByteBuffer>(ErrorCode::NotOpen, "Port is not open");
        }
        
        ByteBuffer buffer(maxBytes);
        size_t bytesRead = 0;
        Duration actualTimeout = timeout.value_or(config_.readTimeout);
        
#if CSERIALPORT_PLATFORM_WINDOWS
        OVERLAPPED ov = {};
        ov.hEvent = readEvent_;
        ResetEvent(readEvent_);
        
        DWORD dwBytesRead = 0;
        BOOL result = ReadFile(handle_, buffer.data(), static_cast<DWORD>(maxBytes), 
                               &dwBytesRead, &ov);
        
        if (!result) {
            if (GetLastError() == ERROR_IO_PENDING) {
                DWORD waitResult = WaitForSingleObject(readEvent_, 
                    static_cast<DWORD>(actualTimeout.count()));
                
                if (waitResult == WAIT_TIMEOUT) {
                    CancelIo(handle_);
                    return Result<ByteBuffer>(ErrorCode::Timeout, "Read timeout");
                } else if (waitResult == WAIT_OBJECT_0) {
                    if (!GetOverlappedResult(handle_, &ov, &dwBytesRead, FALSE)) {
                        statistics_.readErrors++;
                        return Result<ByteBuffer>(ErrorCode::ReadFailed, "Read failed");
                    }
                } else {
                    statistics_.readErrors++;
                    return Result<ByteBuffer>(ErrorCode::ReadFailed, "Wait failed");
                }
            } else {
                statistics_.readErrors++;
                return Result<ByteBuffer>(ErrorCode::ReadFailed, "Read failed");
            }
        }
        
        bytesRead = dwBytesRead;
        
#elif CSERIALPORT_PLATFORM_LINUX
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd_, &readfds);
        
        struct timeval tv;
        tv.tv_sec = actualTimeout.count() / 1000;
        tv.tv_usec = (actualTimeout.count() % 1000) * 1000;
        
        int selectResult = select(fd_ + 1, &readfds, nullptr, nullptr, &tv);
        
        if (selectResult < 0) {
            statistics_.readErrors++;
            return Result<ByteBuffer>(ErrorCode::ReadFailed, "Select failed");
        } else if (selectResult == 0) {
            return Result<ByteBuffer>(ErrorCode::Timeout, "Read timeout");
        }
        
        ssize_t result = ::read(fd_, buffer.data(), maxBytes);
        if (result < 0) {
            statistics_.readErrors++;
            return Result<ByteBuffer>(ErrorCode::ReadFailed, "Read failed");
        }
        
        bytesRead = static_cast<size_t>(result);
#endif
        
        buffer.resize(bytesRead);
        statistics_.bytesReceived += bytesRead;
        
        return Result<ByteBuffer>(std::move(buffer));
    }
    
    Result<ByteBuffer> readExact(size_t exactBytes, std::optional<Duration> timeout) {
        ByteBuffer result;
        result.reserve(exactBytes);
        
        auto startTime = std::chrono::steady_clock::now();
        Duration actualTimeout = timeout.value_or(config_.readTimeout);
        
        while (result.size() < exactBytes) {
            auto elapsed = std::chrono::duration_cast<Duration>(
                std::chrono::steady_clock::now() - startTime);
            
            if (elapsed >= actualTimeout) {
                return Result<ByteBuffer>(ErrorCode::Timeout, "Read exact timeout");
            }
            
            Duration remainingTimeout = actualTimeout - elapsed;
            size_t remaining = exactBytes - result.size();
            
            auto readResult = read(remaining, remainingTimeout);
            if (!readResult) {
                return readResult;
            }
            
            auto& data = readResult.value();
            result.insert(result.end(), data.begin(), data.end());
        }
        
        return Result<ByteBuffer>(std::move(result));
    }
    
    Result<ByteBuffer> readUntil(Byte delimiter, size_t maxBytes, std::optional<Duration> timeout) {
        ByteBuffer result;
        result.reserve(256);
        
        auto startTime = std::chrono::steady_clock::now();
        Duration actualTimeout = timeout.value_or(config_.readTimeout);
        
        while (result.size() < maxBytes) {
            auto elapsed = std::chrono::duration_cast<Duration>(
                std::chrono::steady_clock::now() - startTime);
            
            if (elapsed >= actualTimeout) {
                return Result<ByteBuffer>(ErrorCode::Timeout, "Read until timeout");
            }
            
            Duration remainingTimeout = actualTimeout - elapsed;
            
            auto readResult = read(1, remainingTimeout);
            if (!readResult) {
                if (readResult.error() == ErrorCode::Timeout && !result.empty()) {
                    break;
                }
                return readResult;
            }
            
            auto& data = readResult.value();
            if (!data.empty()) {
                result.push_back(data[0]);
                if (data[0] == delimiter) {
                    break;
                }
            }
        }
        
        return Result<ByteBuffer>(std::move(result));
    }
    
    Result<std::string> readLine(size_t maxBytes, std::optional<Duration> timeout) {
        auto result = readUntil('\n', maxBytes, timeout);
        if (!result) {
            return Result<std::string>(result.error(), result.errorMessage());
        }
        
        auto& data = result.value();
        std::string line(data.begin(), data.end());
        
        while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
            line.pop_back();
        }
        
        return Result<std::string>(std::move(line));
    }
    
    // ========================================================================
    // 同步写入
    // ========================================================================
    
    Result<size_t> write(const Byte* data, size_t size, std::optional<Duration> timeout) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!isOpen_) {
            return Result<size_t>(ErrorCode::NotOpen, "Port is not open");
        }
        
        if (data == nullptr || size == 0) {
            return Result<size_t>(static_cast<size_t>(0));
        }
        
        Duration actualTimeout = timeout.value_or(config_.writeTimeout);
        
#if CSERIALPORT_PLATFORM_WINDOWS
        OVERLAPPED ov = {};
        ov.hEvent = writeEvent_;
        ResetEvent(writeEvent_);
        
        DWORD dwBytesWritten = 0;
        BOOL result = WriteFile(handle_, data, static_cast<DWORD>(size), 
                                &dwBytesWritten, &ov);
        
        if (!result) {
            if (GetLastError() == ERROR_IO_PENDING) {
                DWORD waitResult = WaitForSingleObject(writeEvent_, 
                    static_cast<DWORD>(actualTimeout.count()));
                
                if (waitResult == WAIT_TIMEOUT) {
                    CancelIo(handle_);
                    return Result<size_t>(ErrorCode::Timeout, "Write timeout");
                } else if (waitResult == WAIT_OBJECT_0) {
                    if (!GetOverlappedResult(handle_, &ov, &dwBytesWritten, FALSE)) {
                        statistics_.writeErrors++;
                        return Result<size_t>(ErrorCode::WriteFailed, "Write failed");
                    }
                } else {
                    statistics_.writeErrors++;
                    return Result<size_t>(ErrorCode::WriteFailed, "Wait failed");
                }
            } else {
                statistics_.writeErrors++;
                return Result<size_t>(ErrorCode::WriteFailed, "Write failed");
            }
        }
        
        statistics_.bytesSent += dwBytesWritten;
        return Result<size_t>(static_cast<size_t>(dwBytesWritten));
        
#elif CSERIALPORT_PLATFORM_LINUX
        fd_set writefds;
        FD_ZERO(&writefds);
        FD_SET(fd_, &writefds);
        
        struct timeval tv;
        tv.tv_sec = actualTimeout.count() / 1000;
        tv.tv_usec = (actualTimeout.count() % 1000) * 1000;
        
        int selectResult = select(fd_ + 1, nullptr, &writefds, nullptr, &tv);
        
        if (selectResult < 0) {
            statistics_.writeErrors++;
            return Result<size_t>(ErrorCode::WriteFailed, "Select failed");
        } else if (selectResult == 0) {
            return Result<size_t>(ErrorCode::Timeout, "Write timeout");
        }
        
        ssize_t result = ::write(fd_, data, size);
        if (result < 0) {
            statistics_.writeErrors++;
            return Result<size_t>(ErrorCode::WriteFailed, "Write failed");
        }
        
        statistics_.bytesSent += result;
        return Result<size_t>(static_cast<size_t>(result));
#endif
    }
    
    Result<size_t> write(const ByteBuffer& data, std::optional<Duration> timeout) {
        return write(data.data(), data.size(), timeout);
    }
    
    Result<size_t> write(const std::string& str, std::optional<Duration> timeout) {
        return write(reinterpret_cast<const Byte*>(str.data()), str.size(), timeout);
    }
    
    Result<size_t> writeLine(const std::string& line, std::optional<Duration> timeout) {
        std::string data = line + "\r\n";
        return write(data, timeout);
    }
    
    // ========================================================================
    // 异步操作
    // ========================================================================
    
    std::future<Result<ByteBuffer>> readAsync(size_t maxBytes) {
        return std::async(std::launch::async, [this, maxBytes]() {
            return read(maxBytes, std::nullopt);
        });
    }
    
    std::future<Result<size_t>> writeAsync(ByteBuffer data) {
        return std::async(std::launch::async, [this, data = std::move(data)]() {
            return write(data, std::nullopt);
        });
    }
    
    std::future<Result<size_t>> writeAsync(std::string str) {
        return std::async(std::launch::async, [this, str = std::move(str)]() {
            return write(str, std::nullopt);
        });
    }
    
    // ========================================================================
    // 回调模式
    // ========================================================================
    
    void setDataCallback(DataCallback callback) {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        dataCallback_ = std::move(callback);
    }
    
    void setEventCallback(EventCallback callback) {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        eventCallback_ = std::move(callback);
    }
    
    void setErrorCallback(ErrorCallback callback) {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        errorCallback_ = std::move(callback);
    }
    
    VoidResult startAsyncReceive() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!isOpen_) {
            return VoidResult(ErrorCode::NotOpen, "Port is not open");
        }
        
        if (asyncReceiving_) {
            return VoidResult();
        }
        
        asyncReceiving_ = true;
        
#if CSERIALPORT_PLATFORM_WINDOWS
        ResetEvent(shutdownEvent_);
#endif
        
        receiveThread_ = std::thread(&Impl::receiveThreadFunc, this);
        
        return VoidResult();
    }
    
    void stopAsyncReceive() {
        std::lock_guard<std::mutex> lock(mutex_);
        stopAsyncReceiveInternal();
    }
    
    bool isAsyncReceiving() const noexcept {
        return asyncReceiving_;
    }
    
    // ========================================================================
    // 缓冲区操作
    // ========================================================================
    
    Result<size_t> available() const {
        if (!isOpen_) {
            return Result<size_t>(ErrorCode::NotOpen, "Port is not open");
        }
        
#if CSERIALPORT_PLATFORM_WINDOWS
        COMSTAT comstat;
        DWORD errors;
        if (!ClearCommError(handle_, &errors, &comstat)) {
            return Result<size_t>(ErrorCode::ReadFailed, "Failed to get available bytes");
        }
        return Result<size_t>(static_cast<size_t>(comstat.cbInQue));
        
#elif CSERIALPORT_PLATFORM_LINUX
        int bytes = 0;
        if (ioctl(fd_, FIONREAD, &bytes) < 0) {
            return Result<size_t>(ErrorCode::ReadFailed, "Failed to get available bytes");
        }
        return Result<size_t>(static_cast<size_t>(bytes));
#endif
    }
    
    VoidResult flushInput() {
        if (!isOpen_) {
            return VoidResult(ErrorCode::NotOpen, "Port is not open");
        }
        
#if CSERIALPORT_PLATFORM_WINDOWS
        if (!PurgeComm(handle_, PURGE_RXCLEAR | PURGE_RXABORT)) {
            return VoidResult(ErrorCode::ReadFailed, "Failed to flush input");
        }
#elif CSERIALPORT_PLATFORM_LINUX
        if (tcflush(fd_, TCIFLUSH) != 0) {
            return VoidResult(ErrorCode::ReadFailed, "Failed to flush input");
        }
#endif
        return VoidResult();
    }
    
    VoidResult flushOutput() {
        if (!isOpen_) {
            return VoidResult(ErrorCode::NotOpen, "Port is not open");
        }
        
#if CSERIALPORT_PLATFORM_WINDOWS
        if (!PurgeComm(handle_, PURGE_TXCLEAR | PURGE_TXABORT)) {
            return VoidResult(ErrorCode::WriteFailed, "Failed to flush output");
        }
#elif CSERIALPORT_PLATFORM_LINUX
        if (tcflush(fd_, TCOFLUSH) != 0) {
            return VoidResult(ErrorCode::WriteFailed, "Failed to flush output");
        }
#endif
        return VoidResult();
    }
    
    VoidResult flush() {
        auto result = flushInput();
        if (!result) return result;
        return flushOutput();
    }
    
    // ========================================================================
    // 控制线
    // ========================================================================
    
    VoidResult setDTR(bool state) {
        if (!isOpen_) {
            return VoidResult(ErrorCode::NotOpen, "Port is not open");
        }
        
#if CSERIALPORT_PLATFORM_WINDOWS
        if (!EscapeCommFunction(handle_, state ? SETDTR : CLRDTR)) {
            return VoidResult(ErrorCode::ConfigFailed, "Failed to set DTR");
        }
#elif CSERIALPORT_PLATFORM_LINUX
        int status;
        if (ioctl(fd_, TIOCMGET, &status) < 0) {
            return VoidResult(ErrorCode::ConfigFailed, "Failed to get modem status");
        }
        if (state) {
            status |= TIOCM_DTR;
        } else {
            status &= ~TIOCM_DTR;
        }
        if (ioctl(fd_, TIOCMSET, &status) < 0) {
            return VoidResult(ErrorCode::ConfigFailed, "Failed to set DTR");
        }
#endif
        return VoidResult();
    }
    
    VoidResult setRTS(bool state) {
        if (!isOpen_) {
            return VoidResult(ErrorCode::NotOpen, "Port is not open");
        }
        
#if CSERIALPORT_PLATFORM_WINDOWS
        if (!EscapeCommFunction(handle_, state ? SETRTS : CLRRTS)) {
            return VoidResult(ErrorCode::ConfigFailed, "Failed to set RTS");
        }
#elif CSERIALPORT_PLATFORM_LINUX
        int status;
        if (ioctl(fd_, TIOCMGET, &status) < 0) {
            return VoidResult(ErrorCode::ConfigFailed, "Failed to get modem status");
        }
        if (state) {
            status |= TIOCM_RTS;
        } else {
            status &= ~TIOCM_RTS;
        }
        if (ioctl(fd_, TIOCMSET, &status) < 0) {
            return VoidResult(ErrorCode::ConfigFailed, "Failed to set RTS");
        }
#endif
        return VoidResult();
    }
    
    // ========================================================================
    // 状态
    // ========================================================================
    
    std::string portName() const noexcept {
        return portName_;
    }
    
    PortStatistics statistics() const noexcept {
        return statistics_;
    }
    
    void resetStatistics() noexcept {
        statistics_.reset();
    }
    
    // ========================================================================
    // 静态方法
    // ========================================================================
    
    static std::vector<PortInfo> enumerate() {
        std::vector<PortInfo> ports;
        
#if CSERIALPORT_PLATFORM_WINDOWS
        for (int i = 1; i <= 256; ++i) {
            std::string portName = "COM" + std::to_string(i);
            std::string devicePath = "\\\\.\\" + portName;
            
            HANDLE handle = CreateFileA(
                devicePath.c_str(),
                GENERIC_READ | GENERIC_WRITE,
                0,
                nullptr,
                OPEN_EXISTING,
                0,
                nullptr
            );
            
            if (handle != INVALID_HANDLE_VALUE) {
                CloseHandle(handle);
                PortInfo info;
                info.portName = portName;
                info.isAvailable = true;
                ports.push_back(info);
            } else if (GetLastError() == ERROR_ACCESS_DENIED) {
                PortInfo info;
                info.portName = portName;
                info.isAvailable = false;
                ports.push_back(info);
            }
        }
        
        HKEY hKey;
        if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, 
                          "HARDWARE\\DEVICEMAP\\SERIALCOMM", 
                          0, KEY_READ, &hKey) == ERROR_SUCCESS) {
            
            char valueName[256];
            char valueData[256];
            DWORD valueNameSize, valueDataSize, valueType;
            DWORD index = 0;
            
            while (true) {
                valueNameSize = sizeof(valueName);
                valueDataSize = sizeof(valueData);
                
                LONG result = RegEnumValueA(hKey, index++, valueName, &valueNameSize,
                                            nullptr, &valueType, 
                                            reinterpret_cast<LPBYTE>(valueData), 
                                            &valueDataSize);
                
                if (result != ERROR_SUCCESS) break;
                
                if (valueType == REG_SZ) {
                    std::string portName(valueData);
                    
                    auto it = std::find_if(ports.begin(), ports.end(),
                        [&portName](const PortInfo& p) { return p.portName == portName; });
                    
                    if (it == ports.end()) {
                        PortInfo info;
                        info.portName = portName;
                        info.description = valueName;
                        info.isAvailable = true;
                        ports.push_back(info);
                    } else {
                        it->description = valueName;
                    }
                }
            }
            
            RegCloseKey(hKey);
        }
        
#elif CSERIALPORT_PLATFORM_LINUX
        const char* patterns[] = {
            "/dev/ttyS*",
            "/dev/ttyUSB*",
            "/dev/ttyACM*",
            "/dev/ttyAMA*",
            "/dev/rfcomm*",
            nullptr
        };
        
        for (int i = 0; patterns[i] != nullptr; ++i) {
            glob_t globResult;
            if (glob(patterns[i], GLOB_NOSORT, nullptr, &globResult) == 0) {
                for (size_t j = 0; j < globResult.gl_pathc; ++j) {
                    PortInfo info;
                    info.portName = globResult.gl_pathv[j];
                    
                    int fd = ::open(info.portName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
                    if (fd >= 0) {
                        ::close(fd);
                        info.isAvailable = true;
                    } else {
                        info.isAvailable = (errno == EBUSY);
                    }
                    
                    ports.push_back(info);
                }
                globfree(&globResult);
            }
        }
#endif
        
        return ports;
    }
    
    static bool exists(const std::string& portName) {
#if CSERIALPORT_PLATFORM_WINDOWS
        std::string devicePath = "\\\\.\\" + portName;
        HANDLE handle = CreateFileA(
            devicePath.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,
            nullptr,
            OPEN_EXISTING,
            0,
            nullptr
        );
        
        if (handle != INVALID_HANDLE_VALUE) {
            CloseHandle(handle);
            return true;
        }
        return GetLastError() == ERROR_ACCESS_DENIED;
        
#elif CSERIALPORT_PLATFORM_LINUX
        struct stat st;
        return stat(portName.c_str(), &st) == 0 && S_ISCHR(st.st_mode);
#endif
    }

private:
    void closeInternal() {
#if CSERIALPORT_PLATFORM_WINDOWS
        if (handle_ != INVALID_HANDLE_VALUE) {
            CloseHandle(handle_);
            handle_ = INVALID_HANDLE_VALUE;
        }
        if (readEvent_) {
            CloseHandle(readEvent_);
            readEvent_ = nullptr;
        }
        if (writeEvent_) {
            CloseHandle(writeEvent_);
            writeEvent_ = nullptr;
        }
        if (shutdownEvent_) {
            CloseHandle(shutdownEvent_);
            shutdownEvent_ = nullptr;
        }
#elif CSERIALPORT_PLATFORM_LINUX
        if (fd_ >= 0) {
            tcsetattr(fd_, TCSANOW, &originalTermios_);
            ::close(fd_);
            fd_ = -1;
        }
#endif
        isOpen_ = false;
    }
    
    void stopAsyncReceiveInternal() {
        if (!asyncReceiving_) return;
        
        asyncReceiving_ = false;
        
#if CSERIALPORT_PLATFORM_WINDOWS
        SetEvent(shutdownEvent_);
#endif
        
        if (receiveThread_.joinable()) {
            receiveThread_.join();
        }
    }
    
    VoidResult applyConfig(const SerialConfig& config) {
#if CSERIALPORT_PLATFORM_WINDOWS
        DCB dcb = {};
        dcb.DCBlength = sizeof(DCB);
        
        if (!GetCommState(handle_, &dcb)) {
            return VoidResult(ErrorCode::ConfigFailed, "Failed to get comm state");
        }
        
        dcb.BaudRate = config.getBaudRateValue();
        dcb.ByteSize = static_cast<BYTE>(config.dataBits);
        
        switch (config.stopBits) {
            case StopBits::One:     dcb.StopBits = ONESTOPBIT; break;
            case StopBits::OneHalf: dcb.StopBits = ONE5STOPBITS; break;
            case StopBits::Two:     dcb.StopBits = TWOSTOPBITS; break;
        }
        
        switch (config.parity) {
            case Parity::None:  dcb.Parity = NOPARITY; break;
            case Parity::Odd:   dcb.Parity = ODDPARITY; break;
            case Parity::Even:  dcb.Parity = EVENPARITY; break;
            case Parity::Mark:  dcb.Parity = MARKPARITY; break;
            case Parity::Space: dcb.Parity = SPACEPARITY; break;
        }
        
        dcb.fBinary = TRUE;
        dcb.fParity = (config.parity != Parity::None) ? TRUE : FALSE;
        
        switch (config.flowControl) {
            case FlowControl::None:
                dcb.fOutxCtsFlow = FALSE;
                dcb.fRtsControl = RTS_CONTROL_ENABLE;
                dcb.fOutX = FALSE;
                dcb.fInX = FALSE;
                break;
            case FlowControl::Hardware:
                dcb.fOutxCtsFlow = TRUE;
                dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
                dcb.fOutX = FALSE;
                dcb.fInX = FALSE;
                break;
            case FlowControl::Software:
                dcb.fOutxCtsFlow = FALSE;
                dcb.fRtsControl = RTS_CONTROL_ENABLE;
                dcb.fOutX = TRUE;
                dcb.fInX = TRUE;
                dcb.XonChar = 0x11;
                dcb.XoffChar = 0x13;
                break;
        }
        
        if (!SetCommState(handle_, &dcb)) {
            return VoidResult(ErrorCode::ConfigFailed, "Failed to set comm state");
        }
        
        // 设置超时
        COMMTIMEOUTS timeouts = {};
        timeouts.ReadIntervalTimeout = 50;
        timeouts.ReadTotalTimeoutConstant = static_cast<DWORD>(config.readTimeout.count());
        timeouts.ReadTotalTimeoutMultiplier = 10;
        timeouts.WriteTotalTimeoutConstant = static_cast<DWORD>(config.writeTimeout.count());
        timeouts.WriteTotalTimeoutMultiplier = 10;
        
        if (!SetCommTimeouts(handle_, &timeouts)) {
            return VoidResult(ErrorCode::ConfigFailed, "Failed to set comm timeouts");
        }
        
#elif CSERIALPORT_PLATFORM_LINUX
        struct termios tty = {};
        
        if (tcgetattr(fd_, &tty) != 0) {
            return VoidResult(ErrorCode::ConfigFailed, "Failed to get terminal attributes");
        }
        
        // 设置波特率
        speed_t speed = B9600;
        uint32_t baudValue = config.getBaudRateValue();
        
        switch (baudValue) {
            case 110:    speed = B110; break;
            case 300:    speed = B300; break;
            case 600:    speed = B600; break;
            case 1200:   speed = B1200; break;
            case 2400:   speed = B2400; break;
            case 4800:   speed = B4800; break;
            case 9600:   speed = B9600; break;
            case 19200:  speed = B19200; break;
            case 38400:  speed = B38400; break;
            case 57600:  speed = B57600; break;
            case 115200: speed = B115200; break;
            case 230400: speed = B230400; break;
            case 460800: speed = B460800; break;
            case 921600: speed = B921600; break;
            default:     speed = B9600; break;
        }
        
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);
        
        // 设置数据位
        tty.c_cflag &= ~CSIZE;
        switch (config.dataBits) {
            case DataBits::Five:  tty.c_cflag |= CS5; break;
            case DataBits::Six:   tty.c_cflag |= CS6; break;
            case DataBits::Seven: tty.c_cflag |= CS7; break;
            case DataBits::Eight: tty.c_cflag |= CS8; break;
        }
        
        // 设置停止位
        if (config.stopBits == StopBits::Two) {
            tty.c_cflag |= CSTOPB;
        } else {
            tty.c_cflag &= ~CSTOPB;
        }
        
        // 设置校验位
        switch (config.parity) {
            case Parity::None:
                tty.c_cflag &= ~PARENB;
                break;
            case Parity::Odd:
                tty.c_cflag |= PARENB;
                tty.c_cflag |= PARODD;
                break;
            case Parity::Even:
                tty.c_cflag |= PARENB;
                tty.c_cflag &= ~PARODD;
                break;
            default:
                tty.c_cflag &= ~PARENB;
                break;
        }
        
        // 设置流控制
        switch (config.flowControl) {
            case FlowControl::None:
                tty.c_cflag &= ~CRTSCTS;
                tty.c_iflag &= ~(IXON | IXOFF | IXANY);
                break;
            case FlowControl::Hardware:
                tty.c_cflag |= CRTSCTS;
                tty.c_iflag &= ~(IXON | IXOFF | IXANY);
                break;
            case FlowControl::Software:
                tty.c_cflag &= ~CRTSCTS;
                tty.c_iflag |= (IXON | IXOFF);
                break;
        }
        
        // 其他设置
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;
        
        // 设置超时
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;
        
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            return VoidResult(ErrorCode::ConfigFailed, "Failed to set terminal attributes");
        }
#endif
        
        return VoidResult();
    }
    
    void receiveThreadFunc() {
        ByteBuffer buffer(1024);
        
        while (asyncReceiving_) {
#if CSERIALPORT_PLATFORM_WINDOWS
            HANDLE events[2] = { shutdownEvent_, readEvent_ };
            
            OVERLAPPED ov = {};
            ov.hEvent = readEvent_;
            ResetEvent(readEvent_);
            
            DWORD bytesRead = 0;
            BOOL result = ReadFile(handle_, buffer.data(), static_cast<DWORD>(buffer.size()),
                                   &bytesRead, &ov);
            
            if (!result && GetLastError() == ERROR_IO_PENDING) {
                DWORD waitResult = WaitForMultipleObjects(2, events, FALSE, INFINITE);
                
                if (waitResult == WAIT_OBJECT_0) {
                    // Shutdown event
                    CancelIo(handle_);
                    break;
                } else if (waitResult == WAIT_OBJECT_0 + 1) {
                    if (!GetOverlappedResult(handle_, &ov, &bytesRead, FALSE)) {
                        std::lock_guard<std::mutex> lock(callbackMutex_);
                        if (errorCallback_) {
                            errorCallback_(ErrorCode::ReadFailed, "Read failed in async receive");
                        }
                        continue;
                    }
                }
            }
            
            if (bytesRead > 0) {
                statistics_.bytesReceived += bytesRead;
                
                std::lock_guard<std::mutex> lock(callbackMutex_);
                if (dataCallback_) {
                    dataCallback_(buffer.data(), bytesRead);
                }
                if (eventCallback_) {
                    eventCallback_(EventType::DataReceived);
                }
            }
            
#elif CSERIALPORT_PLATFORM_LINUX
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(fd_, &readfds);
            
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 100000; // 100ms
            
            int selectResult = select(fd_ + 1, &readfds, nullptr, nullptr, &tv);
            
            if (selectResult > 0 && FD_ISSET(fd_, &readfds)) {
                ssize_t bytesRead = ::read(fd_, buffer.data(), buffer.size());
                
                if (bytesRead > 0) {
                    statistics_.bytesReceived += bytesRead;
                    
                    std::lock_guard<std::mutex> lock(callbackMutex_);
                    if (dataCallback_) {
                        dataCallback_(buffer.data(), static_cast<size_t>(bytesRead));
                    }
                    if (eventCallback_) {
                        eventCallback_(EventType::DataReceived);
                    }
                } else if (bytesRead < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                    std::lock_guard<std::mutex> lock(callbackMutex_);
                    if (errorCallback_) {
                        errorCallback_(ErrorCode::ReadFailed, "Read failed in async receive");
                    }
                }
            }
#endif
        }
    }
    
    // 成员变量
    std::string portName_;
    SerialConfig config_;
    std::atomic<bool> isOpen_{false};
    std::atomic<bool> asyncReceiving_{false};
    mutable std::mutex mutex_;
    std::mutex callbackMutex_;
    PortStatistics statistics_;
    
    // 回调函数
    DataCallback dataCallback_;
    EventCallback eventCallback_;
    ErrorCallback errorCallback_;
    
    // 接收线程
    std::thread receiveThread_;
    
#if CSERIALPORT_PLATFORM_WINDOWS
    HANDLE handle_ = INVALID_HANDLE_VALUE;
    HANDLE readEvent_ = nullptr;
    HANDLE writeEvent_ = nullptr;
    HANDLE shutdownEvent_ = nullptr;
#elif CSERIALPORT_PLATFORM_LINUX
    int fd_ = -1;
    struct termios originalTermios_{};
#endif
};

// ============================================================================
// SerialPort 公共接口实现
// ============================================================================

SerialPort::SerialPort() : impl_(std::make_unique<Impl>()) {}

SerialPort::SerialPort(const std::string& portName, const SerialConfig& config)
    : impl_(std::make_unique<Impl>()) {
    impl_->open(portName, config);
}

SerialPort::~SerialPort() = default;

SerialPort::SerialPort(SerialPort&& other) noexcept = default;
SerialPort& SerialPort::operator=(SerialPort&& other) noexcept = default;

std::vector<PortInfo> SerialPort::enumerate() {
    return Impl::enumerate();
}

bool SerialPort::exists(const std::string& portName) {
    return Impl::exists(portName);
}

VoidResult SerialPort::open(const std::string& portName, const SerialConfig& config) {
    return impl_->open(portName, config);
}

VoidResult SerialPort::close() {
    return impl_->close();
}

bool SerialPort::isOpen() const noexcept {
    return impl_->isOpen();
}

SerialConfig SerialPort::config() const noexcept {
    return impl_->config();
}

VoidResult SerialPort::setConfig(const SerialConfig& config) {
    return impl_->setConfig(config);
}

VoidResult SerialPort::setBaudRate(BaudRate baudRate) {
    auto cfg = impl_->config();
    cfg.baudRate = baudRate;
    return impl_->setConfig(cfg);
}

VoidResult SerialPort::setDataBits(DataBits dataBits) {
    auto cfg = impl_->config();
    cfg.dataBits = dataBits;
    return impl_->setConfig(cfg);
}

VoidResult SerialPort::setStopBits(StopBits stopBits) {
    auto cfg = impl_->config();
    cfg.stopBits = stopBits;
    return impl_->setConfig(cfg);
}

VoidResult SerialPort::setParity(Parity parity) {
    auto cfg = impl_->config();
    cfg.parity = parity;
    return impl_->setConfig(cfg);
}

VoidResult SerialPort::setFlowControl(FlowControl flowControl) {
    auto cfg = impl_->config();
    cfg.flowControl = flowControl;
    return impl_->setConfig(cfg);
}

VoidResult SerialPort::setReadTimeout(Duration timeout) {
    auto cfg = impl_->config();
    cfg.readTimeout = timeout;
    return impl_->setConfig(cfg);
}

VoidResult SerialPort::setWriteTimeout(Duration timeout) {
    auto cfg = impl_->config();
    cfg.writeTimeout = timeout;
    return impl_->setConfig(cfg);
}

Result<ByteBuffer> SerialPort::read(size_t maxBytes, std::optional<Duration> timeout) {
    return impl_->read(maxBytes, timeout);
}

Result<ByteBuffer> SerialPort::readExact(size_t exactBytes, std::optional<Duration> timeout) {
    return impl_->readExact(exactBytes, timeout);
}

Result<ByteBuffer> SerialPort::readUntil(Byte delimiter, size_t maxBytes, std::optional<Duration> timeout) {
    return impl_->readUntil(delimiter, maxBytes, timeout);
}

Result<std::string> SerialPort::readLine(size_t maxBytes, std::optional<Duration> timeout) {
    return impl_->readLine(maxBytes, timeout);
}

Result<size_t> SerialPort::write(const Byte* data, size_t size, std::optional<Duration> timeout) {
    return impl_->write(data, size, timeout);
}

Result<size_t> SerialPort::write(const ByteBuffer& data, std::optional<Duration> timeout) {
    return impl_->write(data, timeout);
}

Result<size_t> SerialPort::write(const std::string& str, std::optional<Duration> timeout) {
    return impl_->write(str, timeout);
}

Result<size_t> SerialPort::writeLine(const std::string& line, std::optional<Duration> timeout) {
    return impl_->writeLine(line, timeout);
}

std::future<Result<ByteBuffer>> SerialPort::readAsync(size_t maxBytes) {
    return impl_->readAsync(maxBytes);
}

std::future<Result<size_t>> SerialPort::writeAsync(ByteBuffer data) {
    return impl_->writeAsync(std::move(data));
}

std::future<Result<size_t>> SerialPort::writeAsync(std::string str) {
    return impl_->writeAsync(std::move(str));
}

void SerialPort::setDataCallback(DataCallback callback) {
    impl_->setDataCallback(std::move(callback));
}

void SerialPort::setEventCallback(EventCallback callback) {
    impl_->setEventCallback(std::move(callback));
}

void SerialPort::setErrorCallback(ErrorCallback callback) {
    impl_->setErrorCallback(std::move(callback));
}

VoidResult SerialPort::startAsyncReceive() {
    return impl_->startAsyncReceive();
}

void SerialPort::stopAsyncReceive() {
    impl_->stopAsyncReceive();
}

bool SerialPort::isAsyncReceiving() const noexcept {
    return impl_->isAsyncReceiving();
}

Result<size_t> SerialPort::available() const {
    return impl_->available();
}

VoidResult SerialPort::flushInput() {
    return impl_->flushInput();
}

VoidResult SerialPort::flushOutput() {
    return impl_->flushOutput();
}

VoidResult SerialPort::flush() {
    return impl_->flush();
}

VoidResult SerialPort::setDTR(bool state) {
    return impl_->setDTR(state);
}

VoidResult SerialPort::setRTS(bool state) {
    return impl_->setRTS(state);
}

std::string SerialPort::portName() const noexcept {
    return impl_->portName();
}

PortStatistics SerialPort::statistics() const noexcept {
    return impl_->statistics();
}

void SerialPort::resetStatistics() noexcept {
    impl_->resetStatistics();
}

} // namespace csp