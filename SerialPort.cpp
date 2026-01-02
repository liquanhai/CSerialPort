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
#include <thread>

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
        if (!readEvent_) {
            closeInternal();
            return VoidResult(ErrorCode::OpenFailed, "Failed to create read event");
        }

        writeEvent_ = CreateEvent(nullptr, TRUE, FALSE, nullptr);
        if (!writeEvent_) {
            closeInternal();
            return VoidResult(ErrorCode::OpenFailed, "Failed to create write event");
        }

        shutdownEvent_ = CreateEvent(nullptr, TRUE, FALSE, nullptr);
        if (!shutdownEvent_) {
            closeInternal();
            return VoidResult(ErrorCode::OpenFailed, "Failed to create shutdown event");
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
    
    Result<ByteBuffer> readInternal(size_t maxBytes, std::optional<Duration> timeout) {
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
                        statistics_.readErrors.fetch_add(1, std::memory_order_relaxed);
                        return Result<ByteBuffer>(ErrorCode::ReadFailed, "Read failed");
                    }
                } else {
                    statistics_.readErrors.fetch_add(1, std::memory_order_relaxed);
                    return Result<ByteBuffer>(ErrorCode::ReadFailed, "Wait failed");
                }
            } else {
                statistics_.readErrors.fetch_add(1, std::memory_order_relaxed);
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
            statistics_.readErrors.fetch_add(1, std::memory_order_relaxed);
            return Result<ByteBuffer>(ErrorCode::ReadFailed, "Select failed");
        } else if (selectResult == 0) {
            return Result<ByteBuffer>(ErrorCode::Timeout, "Read timeout");
        }
        
        ssize_t result = ::read(fd_, buffer.data(), maxBytes);
        if (result < 0) {
            statistics_.readErrors.fetch_add(1, std::memory_order_relaxed);
            return Result<ByteBuffer>(ErrorCode::ReadFailed, "Read failed");
        }
        
        bytesRead = static_cast<size_t>(result);
#endif
        
        buffer.resize(bytesRead);
        statistics_.bytesReceived.fetch_add(bytesRead, std::memory_order_relaxed);
        
        return Result<ByteBuffer>(std::move(buffer));
    }
    
    Result<ByteBuffer> read(size_t maxBytes, std::optional<Duration> timeout) {
        std::lock_guard<std::mutex> lock(mutex_);
        return readInternal(maxBytes, timeout);
    }
    
    Result<ByteBuffer> readExact(size_t exactBytes, std::optional<Duration> timeout) {
        std::lock_guard<std::mutex> lock(mutex_);
        
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
            
            auto readResult = readInternal(remaining, remainingTimeout);
            if (!readResult) {
                return readResult;
            }
            
            auto& data = readResult.value();
            result.insert(result.end(), data.begin(), data.end());
        }
        
        return Result<ByteBuffer>(std::move(result));
    }
    
    Result<ByteBuffer> readUntilInternal(Byte delimiter, size_t maxBytes, std::optional<Duration> timeout) {
        ByteBuffer result;
        result.reserve(std::min(maxBytes, static_cast<size_t>(256)));

        // 内部缓冲区用于批量读取
        ByteBuffer readBuffer(128);
        size_t bufferPos = 0;
        size_t bufferLen = 0;

        auto startTime = std::chrono::steady_clock::now();
        Duration actualTimeout = timeout.value_or(config_.readTimeout);

        while (result.size() < maxBytes) {
            auto elapsed = std::chrono::duration_cast<Duration>(
                std::chrono::steady_clock::now() - startTime);

            if (elapsed >= actualTimeout) {
                // 如果已经读取了一些数据，返回它们而不是报错
                if (!result.empty()) {
                    break;
                }
                return Result<ByteBuffer>(ErrorCode::Timeout, "Read until timeout");
            }

            // 如果缓冲区中还有数据，先处理缓冲区
            if (bufferPos < bufferLen) {
                while (bufferPos < bufferLen && result.size() < maxBytes) {
                    Byte b = readBuffer[bufferPos++];
                    result.push_back(b);
                    if (b == delimiter) {
                        return Result<ByteBuffer>(std::move(result));
                    }
                }
                continue;
            }

            // 缓冲区为空，读取更多数据
            Duration remainingTimeout = actualTimeout - elapsed;
            size_t bytesToRead = std::min(readBuffer.size(), maxBytes - result.size());

            auto readResult = readInternal(bytesToRead, remainingTimeout);
            if (!readResult) {
                if (readResult.error() == ErrorCode::Timeout && !result.empty()) {
                    break;
                }
                return readResult;
            }

            auto& data = readResult.value();
            if (data.empty()) {
                continue;
            }

            // 在读取的数据中查找分隔符
            for (size_t i = 0; i < data.size() && result.size() < maxBytes; ++i) {
                result.push_back(data[i]);
                if (data[i] == delimiter) {
                    return Result<ByteBuffer>(std::move(result));
                }
            }
        }

        return Result<ByteBuffer>(std::move(result));
    }
    
    Result<ByteBuffer> readUntil(Byte delimiter, size_t maxBytes, std::optional<Duration> timeout) {
        std::lock_guard<std::mutex> lock(mutex_);
        return readUntilInternal(delimiter, maxBytes, timeout);
    }
    
    Result<std::string> readLine(size_t maxBytes, std::optional<Duration> timeout) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto result = readUntilInternal('\n', maxBytes, timeout);
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
                        statistics_.writeErrors.fetch_add(1, std::memory_order_relaxed);
                        return Result<size_t>(ErrorCode::WriteFailed, "Write failed");
                    }
                } else {
                    statistics_.writeErrors.fetch_add(1, std::memory_order_relaxed);
                    return Result<size_t>(ErrorCode::WriteFailed, "Wait failed");
                }
            } else {
                statistics_.writeErrors.fetch_add(1, std::memory_order_relaxed);
                return Result<size_t>(ErrorCode::WriteFailed, "Write failed");
            }
        }
        
        statistics_.bytesSent.fetch_add(dwBytesWritten, std::memory_order_relaxed);
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
            statistics_.writeErrors.fetch_add(1, std::memory_order_relaxed);
            return Result<size_t>(ErrorCode::WriteFailed, "Select failed");
        } else if (selectResult == 0) {
            return Result<size_t>(ErrorCode::Timeout, "Write timeout");
        }
        
        ssize_t result = ::write(fd_, data, size);
        if (result < 0) {
            statistics_.writeErrors.fetch_add(1, std::memory_order_relaxed);
            return Result<size_t>(ErrorCode::WriteFailed, "Write failed");
        }
        
        statistics_.bytesSent.fetch_add(result, std::memory_order_relaxed);
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

    Result<bool> getCTS() const {
        if (!isOpen_) {
            return Result<bool>(ErrorCode::NotOpen, "Port is not open");
        }

#if CSERIALPORT_PLATFORM_WINDOWS
        DWORD status;
        if (!GetCommModemStatus(handle_, &status)) {
            return Result<bool>(ErrorCode::ReadFailed, "Failed to get modem status");
        }
        return Result<bool>((status & MS_CTS_ON) != 0);
#elif CSERIALPORT_PLATFORM_LINUX
        int status;
        if (ioctl(fd_, TIOCMGET, &status) < 0) {
            return Result<bool>(ErrorCode::ReadFailed, "Failed to get modem status");
        }
        return Result<bool>((status & TIOCM_CTS) != 0);
#endif
    }

    Result<bool> getDSR() const {
        if (!isOpen_) {
            return Result<bool>(ErrorCode::NotOpen, "Port is not open");
        }

#if CSERIALPORT_PLATFORM_WINDOWS
        DWORD status;
        if (!GetCommModemStatus(handle_, &status)) {
            return Result<bool>(ErrorCode::ReadFailed, "Failed to get modem status");
        }
        return Result<bool>((status & MS_DSR_ON) != 0);
#elif CSERIALPORT_PLATFORM_LINUX
        int status;
        if (ioctl(fd_, TIOCMGET, &status) < 0) {
            return Result<bool>(ErrorCode::ReadFailed, "Failed to get modem status");
        }
        return Result<bool>((status & TIOCM_DSR) != 0);
#endif
    }

    Result<bool> getCD() const {
        if (!isOpen_) {
            return Result<bool>(ErrorCode::NotOpen, "Port is not open");
        }

#if CSERIALPORT_PLATFORM_WINDOWS
        DWORD status;
        if (!GetCommModemStatus(handle_, &status)) {
            return Result<bool>(ErrorCode::ReadFailed, "Failed to get modem status");
        }
        return Result<bool>((status & MS_RLSD_ON) != 0);
#elif CSERIALPORT_PLATFORM_LINUX
        int status;
        if (ioctl(fd_, TIOCMGET, &status) < 0) {
            return Result<bool>(ErrorCode::ReadFailed, "Failed to get modem status");
        }
        return Result<bool>((status & TIOCM_CD) != 0);
#endif
    }

    Result<bool> getRI() const {
        if (!isOpen_) {
            return Result<bool>(ErrorCode::NotOpen, "Port is not open");
        }

#if CSERIALPORT_PLATFORM_WINDOWS
        DWORD status;
        if (!GetCommModemStatus(handle_, &status)) {
            return Result<bool>(ErrorCode::ReadFailed, "Failed to get modem status");
        }
        return Result<bool>((status & MS_RING_ON) != 0);
#elif CSERIALPORT_PLATFORM_LINUX
        int status;
        if (ioctl(fd_, TIOCMGET, &status) < 0) {
            return Result<bool>(ErrorCode::ReadFailed, "Failed to get modem status");
        }
        return Result<bool>((status & TIOCM_RI) != 0);
#endif
    }

    VoidResult sendBreak(Duration duration) {
        if (!isOpen_) {
            return VoidResult(ErrorCode::NotOpen, "Port is not open");
        }

#if CSERIALPORT_PLATFORM_WINDOWS
        if (!SetCommBreak(handle_)) {
            return VoidResult(ErrorCode::WriteFailed, "Failed to set break");
        }
        std::this_thread::sleep_for(duration);
        if (!ClearCommBreak(handle_)) {
            return VoidResult(ErrorCode::WriteFailed, "Failed to clear break");
        }
#elif CSERIALPORT_PLATFORM_LINUX
        // Linux tcsendbreak: duration 0 sends break for 0.25-0.5 seconds
        // For custom duration, we use TIOCSBRK/TIOCCBRK
        if (duration.count() <= 0) {
            if (tcsendbreak(fd_, 0) < 0) {
                return VoidResult(ErrorCode::WriteFailed, "Failed to send break");
            }
        } else {
            if (ioctl(fd_, TIOCSBRK, 0) < 0) {
                return VoidResult(ErrorCode::WriteFailed, "Failed to set break");
            }
            std::this_thread::sleep_for(duration);
            if (ioctl(fd_, TIOCCBRK, 0) < 0) {
                return VoidResult(ErrorCode::WriteFailed, "Failed to clear break");
            }
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
        // 使用 SetupAPI 枚举串口（更高效）
        GUID guid = {0x86E0D1E0L, 0x8089, 0x11D0, {0x9C, 0xE4, 0x08, 0x00, 0x3E, 0x30, 0x1F, 0x73}};

        HDEVINFO hDevInfo = SetupDiGetClassDevs(&guid, nullptr, nullptr, DIGCF_PRESENT);

        if (hDevInfo != INVALID_HANDLE_VALUE) {
            SP_DEVINFO_DATA devInfoData;
            devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);

            for (DWORD i = 0; SetupDiEnumDeviceInfo(hDevInfo, i, &devInfoData); ++i) {
                // 获取端口名称
                HKEY hKey = SetupDiOpenDevRegKey(hDevInfo, &devInfoData, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
                if (hKey != INVALID_HANDLE_VALUE) {
                    char portName[256] = {0};
                    DWORD portNameSize = sizeof(portName);
                    DWORD type = 0;

                    if (RegQueryValueExA(hKey, "PortName", nullptr, &type,
                                         reinterpret_cast<LPBYTE>(portName), &portNameSize) == ERROR_SUCCESS) {
                        // 只处理 COM 端口
                        if (strncmp(portName, "COM", 3) == 0) {
                            PortInfo info;
                            info.portName = portName;

                            // 获取设备描述
                            char description[256] = {0};
                            DWORD descSize = sizeof(description);
                            if (SetupDiGetDeviceRegistryPropertyA(hDevInfo, &devInfoData, SPDRP_FRIENDLYNAME,
                                                                   nullptr, reinterpret_cast<PBYTE>(description),
                                                                   descSize, nullptr)) {
                                info.description = description;
                            } else if (SetupDiGetDeviceRegistryPropertyA(hDevInfo, &devInfoData, SPDRP_DEVICEDESC,
                                                                          nullptr, reinterpret_cast<PBYTE>(description),
                                                                          descSize, nullptr)) {
                                info.description = description;
                            }

                            // 获取硬件 ID
                            char hardwareId[256] = {0};
                            DWORD hwIdSize = sizeof(hardwareId);
                            if (SetupDiGetDeviceRegistryPropertyA(hDevInfo, &devInfoData, SPDRP_HARDWAREID,
                                                                   nullptr, reinterpret_cast<PBYTE>(hardwareId),
                                                                   hwIdSize, nullptr)) {
                                info.hardwareId = hardwareId;
                            }

                            // 检查端口是否可用
                            std::string devicePath = "\\\\.\\" + info.portName;
                            HANDLE handle = CreateFileA(devicePath.c_str(), GENERIC_READ | GENERIC_WRITE,
                                                        0, nullptr, OPEN_EXISTING, 0, nullptr);
                            if (handle != INVALID_HANDLE_VALUE) {
                                CloseHandle(handle);
                                info.isAvailable = true;
                            } else {
                                info.isAvailable = (GetLastError() != ERROR_FILE_NOT_FOUND);
                            }

                            ports.push_back(info);
                        }
                    }
                    RegCloseKey(hKey);
                }
            }

            SetupDiDestroyDeviceInfoList(hDevInfo);
        }

        // 如果 SetupAPI 没有找到任何端口，回退到注册表方法
        if (ports.empty()) {
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
                        PortInfo info;
                        info.portName = valueData;
                        info.description = valueName;

                        // 检查端口是否可用
                        std::string devicePath = "\\\\.\\" + info.portName;
                        HANDLE handle = CreateFileA(devicePath.c_str(), GENERIC_READ | GENERIC_WRITE,
                                                    0, nullptr, OPEN_EXISTING, 0, nullptr);
                        if (handle != INVALID_HANDLE_VALUE) {
                            CloseHandle(handle);
                            info.isAvailable = true;
                        } else {
                            info.isAvailable = (GetLastError() != ERROR_FILE_NOT_FOUND);
                        }

                        ports.push_back(info);
                    }
                }

                RegCloseKey(hKey);
            }
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
        // Linux 平台参数验证
        if (config.stopBits == StopBits::OneHalf) {
            return VoidResult(ErrorCode::InvalidParameter,
                "1.5 stop bits is not supported on Linux");
        }

        if (config.parity == Parity::Mark || config.parity == Parity::Space) {
            return VoidResult(ErrorCode::InvalidParameter,
                "Mark and Space parity are not supported on Linux");
        }

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
            default:
                return VoidResult(ErrorCode::InvalidParameter,
                    "Unsupported baud rate: " + std::to_string(baudValue));
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

        // 设置停止位（已验证不是 1.5 停止位）
        if (config.stopBits == StopBits::Two) {
            tty.c_cflag |= CSTOPB;
        } else {
            tty.c_cflag &= ~CSTOPB;
        }

        // 设置校验位（已验证不是 Mark/Space）
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
                        ErrorCallback errorCb;
                        {
                            std::lock_guard<std::mutex> lock(callbackMutex_);
                            errorCb = errorCallback_;
                        }
                        if (errorCb) {
                            errorCb(ErrorCode::ReadFailed, "Read failed in async receive");
                        }
                        continue;
                    }
                } else {
                    // Wait failed or timeout
                    continue;
                }
            } else if (result) {
                // Read completed immediately
                if (!GetOverlappedResult(handle_, &ov, &bytesRead, FALSE)) {
                    ErrorCallback errorCb;
                    {
                        std::lock_guard<std::mutex> lock(callbackMutex_);
                        errorCb = errorCallback_;
                    }
                    if (errorCb) {
                        errorCb(ErrorCode::ReadFailed, "Read failed in async receive");
                    }
                    continue;
                }
            } else {
                // Read failed immediately
                if (GetLastError() != ERROR_IO_PENDING) {
                    ErrorCallback errorCb;
                    {
                        std::lock_guard<std::mutex> lock(callbackMutex_);
                        errorCb = errorCallback_;
                    }
                    if (errorCb) {
                        errorCb(ErrorCode::ReadFailed, "Read failed in async receive");
                    }
                    // Small delay to prevent busy loop on persistent error
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
            }

            if (bytesRead > 0) {
                statistics_.bytesReceived.fetch_add(bytesRead, std::memory_order_relaxed);

                DataCallback dataCb;
                EventCallback eventCb;
                {
                    std::lock_guard<std::mutex> lock(callbackMutex_);
                    dataCb = dataCallback_;
                    eventCb = eventCallback_;
                }
                if (dataCb) {
                    dataCb(buffer.data(), bytesRead);
                }
                if (eventCb) {
                    eventCb(EventType::DataReceived);
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

            if (!asyncReceiving_) {
                // Check if we should exit immediately
                break;
            }

            if (selectResult > 0 && FD_ISSET(fd_, &readfds)) {
                ssize_t bytesRead = ::read(fd_, buffer.data(), buffer.size());

                if (bytesRead > 0) {
                    statistics_.bytesReceived.fetch_add(bytesRead, std::memory_order_relaxed);

                    DataCallback dataCb;
                    EventCallback eventCb;
                    {
                        std::lock_guard<std::mutex> lock(callbackMutex_);
                        dataCb = dataCallback_;
                        eventCb = eventCallback_;
                    }
                    if (dataCb) {
                        dataCb(buffer.data(), static_cast<size_t>(bytesRead));
                    }
                    if (eventCb) {
                        eventCb(EventType::DataReceived);
                    }
                } else if (bytesRead == 0) {
                    // EOF - port closed
                    ErrorCallback errorCb;
                    {
                        std::lock_guard<std::mutex> lock(callbackMutex_);
                        errorCb = errorCallback_;
                    }
                    if (errorCb) {
                        errorCb(ErrorCode::ReadFailed, "Port closed unexpectedly");
                    }
                    break;
                } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    // Real error
                    ErrorCallback errorCb;
                    {
                        std::lock_guard<std::mutex> lock(callbackMutex_);
                        errorCb = errorCallback_;
                    }
                    if (errorCb) {
                        errorCb(ErrorCode::ReadFailed, "Read failed in async receive");
                    }
                    // Small delay to prevent busy loop
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            } else if (selectResult < 0) {
                // Select error
                if (errno != EINTR) {
                    ErrorCallback errorCb;
                    {
                        std::lock_guard<std::mutex> lock(callbackMutex_);
                        errorCb = errorCallback_;
                    }
                    if (errorCb) {
                        errorCb(ErrorCode::ReadFailed, "Select failed in async receive");
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
            // selectResult == 0 means timeout, continue loop
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

Result<bool> SerialPort::getCTS() const {
    return impl_->getCTS();
}

Result<bool> SerialPort::getDSR() const {
    return impl_->getDSR();
}

Result<bool> SerialPort::getCD() const {
    return impl_->getCD();
}

Result<bool> SerialPort::getRI() const {
    return impl_->getRI();
}

VoidResult SerialPort::sendBreak(Duration duration) {
    return impl_->sendBreak(duration);
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