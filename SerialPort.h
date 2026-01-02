/**
 * @file SerialPort.h
 * @brief 现代C++跨平台串口通信库
 * @author CSerialPort Team
 * @version 3.0.0
 * @date 2026-01-01
 * 
 * @details
 * 本库使用现代C++17特性重写，支持Windows和Linux双平台。
 * 
 * 主要特性：
 * - 跨平台支持（Windows/Linux）
 * - RAII资源管理
 * - 线程安全设计
 * - 异步I/O操作
 * - 回调机制
 * - 智能指针管理
 * 
 * @copyright MIT License
 */

#ifndef CSERIAL_PORT_H
#define CSERIAL_PORT_H

// ============================================================================
// 平台检测
// ============================================================================
#if defined(_WIN32) || defined(_WIN64)
    #define CSERIALPORT_PLATFORM_WINDOWS 1
    #define CSERIALPORT_PLATFORM_LINUX 0
#elif defined(__linux__) || defined(__unix__) || defined(__APPLE__)
    #define CSERIALPORT_PLATFORM_WINDOWS 0
    #define CSERIALPORT_PLATFORM_LINUX 1
#else
    #error "Unsupported platform"
#endif

// ============================================================================
// 标准库头文件
// ============================================================================
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <optional>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <future>
#include <queue>
#include <cstdint>
#include <stdexcept>

// ============================================================================
// 平台特定头文件
// ============================================================================
#if CSERIALPORT_PLATFORM_WINDOWS
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
    #include <windows.h>
#elif CSERIALPORT_PLATFORM_LINUX
    #include <termios.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <sys/ioctl.h>
    #include <sys/select.h>
    #include <errno.h>
    #include <dirent.h>
#endif

namespace csp {

// ============================================================================
// 版本信息
// ============================================================================
constexpr int VERSION_MAJOR = 3;
constexpr int VERSION_MINOR = 0;
constexpr int VERSION_PATCH = 0;
constexpr const char* VERSION_STRING = "3.0.0";

// ============================================================================
// 枚举类型定义
// ============================================================================

/**
 * @brief 波特率枚举
 */
enum class BaudRate : uint32_t {
    BR_110    = 110,
    BR_300    = 300,
    BR_600    = 600,
    BR_1200   = 1200,
    BR_2400   = 2400,
    BR_4800   = 4800,
    BR_9600   = 9600,
    BR_14400  = 14400,
    BR_19200  = 19200,
    BR_38400  = 38400,
    BR_57600  = 57600,
    BR_115200 = 115200,
    BR_230400 = 230400,
    BR_460800 = 460800,
    BR_921600 = 921600,
    Custom    = 0
};

/**
 * @brief 数据位枚举
 */
enum class DataBits : uint8_t {
    Five  = 5,
    Six   = 6,
    Seven = 7,
    Eight = 8
};

/**
 * @brief 停止位枚举
 */
enum class StopBits : uint8_t {
    One     = 1,
    OneHalf = 2,
    Two     = 3
};

/**
 * @brief 校验位枚举
 */
enum class Parity : uint8_t {
    None  = 0,
    Odd   = 1,
    Even  = 2,
    Mark  = 3,
    Space = 4
};

/**
 * @brief 流控制枚举
 */
enum class FlowControl : uint8_t {
    None     = 0,
    Hardware = 1,
    Software = 2
};

/**
 * @brief 错误码枚举
 */
enum class ErrorCode : int {
    Success = 0,
    Unknown = 1,
    InvalidParameter = 2,
    NotOpen = 3,
    AlreadyOpen = 4,
    Timeout = 5,
    OpenFailed = 100,
    CloseFailed = 101,
    PortNotFound = 102,
    PermissionDenied = 103,
    PortBusy = 104,
    ConfigFailed = 200,
    ReadFailed = 300,
    WriteFailed = 301,
    BufferOverflow = 302
};

/**
 * @brief 串口事件类型
 */
enum class EventType : uint8_t {
    DataReceived = 1,
    DataSent = 2,
    Error = 3,
    BreakDetected = 4,
    CtsChanged = 5,
    DsrChanged = 6
};

// ============================================================================
// 类型别名
// ============================================================================
using Byte = uint8_t;
using ByteBuffer = std::vector<Byte>;
using Duration = std::chrono::milliseconds;
using DataCallback = std::function<void(const Byte* data, size_t size)>;
using EventCallback = std::function<void(EventType event)>;
using ErrorCallback = std::function<void(ErrorCode error, const std::string& message)>;

// ============================================================================
// 结构体定义
// ============================================================================

/**
 * @brief 串口配置结构体
 */
struct SerialConfig {
    BaudRate baudRate = BaudRate::BR_9600;
    DataBits dataBits = DataBits::Eight;
    StopBits stopBits = StopBits::One;
    Parity parity = Parity::None;
    FlowControl flowControl = FlowControl::None;
    uint32_t customBaudRate = 0;
    Duration readTimeout = Duration(1000);
    Duration writeTimeout = Duration(1000);
    size_t readBufferSize = 4096;
    size_t writeBufferSize = 4096;
    
    uint32_t getBaudRateValue() const noexcept {
        return (baudRate == BaudRate::Custom) ? customBaudRate : static_cast<uint32_t>(baudRate);
    }
    
    static SerialConfig defaultConfig() noexcept { return SerialConfig{}; }
    
    static SerialConfig config_9600_8N1() noexcept {
        SerialConfig cfg;
        cfg.baudRate = BaudRate::BR_9600;
        cfg.dataBits = DataBits::Eight;
        cfg.stopBits = StopBits::One;
        cfg.parity = Parity::None;
        return cfg;
    }
    
    static SerialConfig config_115200_8N1() noexcept {
        SerialConfig cfg;
        cfg.baudRate = BaudRate::BR_115200;
        cfg.dataBits = DataBits::Eight;
        cfg.stopBits = StopBits::One;
        cfg.parity = Parity::None;
        return cfg;
    }
};

/**
 * @brief 串口信息结构体
 */
struct PortInfo {
    std::string portName;
    std::string description;
    std::string hardwareId;
    bool isAvailable = true;
    
    std::string displayName() const {
        return description.empty() ? portName : portName + " - " + description;
    }
};

/**
 * @brief 串口统计信息
 */
struct PortStatistics {
    uint64_t bytesReceived = 0;
    uint64_t bytesSent = 0;
    uint64_t readErrors = 0;
    uint64_t writeErrors = 0;
    
    void reset() noexcept {
        bytesReceived = bytesSent = readErrors = writeErrors = 0;
    }
};

// ============================================================================
// 错误处理类
// ============================================================================

/**
 * @brief 串口错误类
 */
class SerialError : public std::runtime_error {
public:
    explicit SerialError(ErrorCode code, const std::string& message = "")
        : std::runtime_error(message.empty() ? errorCodeToString(code) : message)
        , code_(code) {}
    
    ErrorCode code() const noexcept { return code_; }
    
    static std::string errorCodeToString(ErrorCode code) {
        switch (code) {
            case ErrorCode::Success: return "Success";
            case ErrorCode::Unknown: return "Unknown error";
            case ErrorCode::InvalidParameter: return "Invalid parameter";
            case ErrorCode::NotOpen: return "Port not open";
            case ErrorCode::AlreadyOpen: return "Port already open";
            case ErrorCode::Timeout: return "Operation timeout";
            case ErrorCode::OpenFailed: return "Failed to open port";
            case ErrorCode::CloseFailed: return "Failed to close port";
            case ErrorCode::PortNotFound: return "Port not found";
            case ErrorCode::PermissionDenied: return "Permission denied";
            case ErrorCode::PortBusy: return "Port is busy";
            case ErrorCode::ConfigFailed: return "Configuration failed";
            case ErrorCode::ReadFailed: return "Read failed";
            case ErrorCode::WriteFailed: return "Write failed";
            case ErrorCode::BufferOverflow: return "Buffer overflow";
            default: return "Unknown error code";
        }
    }

private:
    ErrorCode code_;
};

// ============================================================================
// 结果类型
// ============================================================================

/**
 * @brief 操作结果类型
 */
template<typename T>
class Result {
public:
    Result(const T& value) : value_(value), hasValue_(true) {}
    Result(T&& value) : value_(std::move(value)), hasValue_(true) {}
    Result(ErrorCode code, const std::string& msg = "")
        : error_(code), errorMsg_(msg), hasValue_(false) {}

    // 拷贝构造函数
    Result(const Result& other)
        : value_(other.value_), error_(other.error_), errorMsg_(other.errorMsg_), hasValue_(other.hasValue_) {}

    // 移动构造函数
    Result(Result&& other) noexcept
        : value_(std::move(other.value_)), error_(other.error_), errorMsg_(std::move(other.errorMsg_)), hasValue_(other.hasValue_) {}

    // 拷贝赋值运算符
    Result& operator=(const Result& other) {
        if (this != &other) {
            value_ = other.value_;
            error_ = other.error_;
            errorMsg_ = other.errorMsg_;
            hasValue_ = other.hasValue_;
        }
        return *this;
    }

    // 移动赋值运算符
    Result& operator=(Result&& other) noexcept {
        if (this != &other) {
            value_ = std::move(other.value_);
            error_ = other.error_;
            errorMsg_ = std::move(other.errorMsg_);
            hasValue_ = other.hasValue_;
        }
        return *this;
    }

    bool hasValue() const noexcept { return hasValue_; }
    explicit operator bool() const noexcept { return hasValue_; }

    T& value() & {
        if (!hasValue_) throw SerialError(error_, errorMsg_);
        return value_;
    }
    const T& value() const& {
        if (!hasValue_) throw SerialError(error_, errorMsg_);
        return value_;
    }
    T&& value() && {
        if (!hasValue_) throw SerialError(error_, errorMsg_);
        return std::move(value_);
    }

    template<typename U>
    T valueOr(U&& defaultValue) const& {
        return hasValue_ ? value_ : static_cast<T>(std::forward<U>(defaultValue));
    }

    ErrorCode error() const noexcept { return error_; }
    const std::string& errorMessage() const noexcept { return errorMsg_; }

    T& operator*() & { return value(); }
    const T& operator*() const& { return value(); }
    T* operator->() { return &value(); }
    const T* operator->() const { return &value(); }

private:
    T value_{};
    ErrorCode error_ = ErrorCode::Success;
    std::string errorMsg_;
    bool hasValue_;
};

/**
 * @brief 无返回值的操作结果特化
 */
template<>
class Result<void> {
public:
    Result() : hasValue_(true) {}
    Result(ErrorCode code, const std::string& msg = "")
        : error_(code), errorMsg_(msg), hasValue_(false) {}

    // 拷贝构造函数
    Result(const Result& other)
        : error_(other.error_), errorMsg_(other.errorMsg_), hasValue_(other.hasValue_) {}

    // 移动构造函数
    Result(Result&& other) noexcept
        : error_(other.error_), errorMsg_(std::move(other.errorMsg_)), hasValue_(other.hasValue_) {}

    // 拷贝赋值运算符
    Result& operator=(const Result& other) {
        if (this != &other) {
            error_ = other.error_;
            errorMsg_ = other.errorMsg_;
            hasValue_ = other.hasValue_;
        }
        return *this;
    }

    // 移动赋值运算符
    Result& operator=(Result&& other) noexcept {
        if (this != &other) {
            error_ = other.error_;
            errorMsg_ = std::move(other.errorMsg_);
            hasValue_ = other.hasValue_;
        }
        return *this;
    }

    bool hasValue() const noexcept { return hasValue_; }
    explicit operator bool() const noexcept { return hasValue_; }
    ErrorCode error() const noexcept { return error_; }
    const std::string& errorMessage() const noexcept { return errorMsg_; }

private:
    ErrorCode error_ = ErrorCode::Success;
    std::string errorMsg_;
    bool hasValue_;
};

using VoidResult = Result<void>;

// ============================================================================
// 串口类
// ============================================================================

/**
 * @brief 现代C++跨平台串口类
 */
class SerialPort {
public:
    SerialPort();
    explicit SerialPort(const std::string& portName, const SerialConfig& config = SerialConfig::defaultConfig());
    ~SerialPort();
    
    // 禁止拷贝
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;
    
    // 允许移动
    SerialPort(SerialPort&& other) noexcept;
    SerialPort& operator=(SerialPort&& other) noexcept;
    
    // 静态方法
    static std::vector<PortInfo> enumerate();
    static bool exists(const std::string& portName);
    static std::string version() noexcept { return VERSION_STRING; }
    
    // 打开/关闭
    VoidResult open(const std::string& portName, const SerialConfig& config = SerialConfig::defaultConfig());
    VoidResult close();
    bool isOpen() const noexcept;
    
    // 配置
    SerialConfig config() const noexcept;
    VoidResult setConfig(const SerialConfig& config);
    VoidResult setBaudRate(BaudRate baudRate);
    VoidResult setDataBits(DataBits dataBits);
    VoidResult setStopBits(StopBits stopBits);
    VoidResult setParity(Parity parity);
    VoidResult setFlowControl(FlowControl flowControl);
    VoidResult setReadTimeout(Duration timeout);
    VoidResult setWriteTimeout(Duration timeout);
    
    // 同步读写
    Result<ByteBuffer> read(size_t maxBytes, std::optional<Duration> timeout = std::nullopt);
    Result<ByteBuffer> readExact(size_t exactBytes, std::optional<Duration> timeout = std::nullopt);
    Result<ByteBuffer> readUntil(Byte delimiter, size_t maxBytes = 4096, std::optional<Duration> timeout = std::nullopt);
    Result<std::string> readLine(size_t maxBytes = 4096, std::optional<Duration> timeout = std::nullopt);
    
    Result<size_t> write(const Byte* data, size_t size, std::optional<Duration> timeout = std::nullopt);
    Result<size_t> write(const ByteBuffer& data, std::optional<Duration> timeout = std::nullopt);
    Result<size_t> write(const std::string& str, std::optional<Duration> timeout = std::nullopt);
    Result<size_t> writeLine(const std::string& line, std::optional<Duration> timeout = std::nullopt);
    
    // 异步读写
    std::future<Result<ByteBuffer>> readAsync(size_t maxBytes);
    std::future<Result<size_t>> writeAsync(ByteBuffer data);
    std::future<Result<size_t>> writeAsync(std::string str);
    
    // 回调模式
    void setDataCallback(DataCallback callback);
    void setEventCallback(EventCallback callback);
    void setErrorCallback(ErrorCallback callback);
    VoidResult startAsyncReceive();
    void stopAsyncReceive();
    bool isAsyncReceiving() const noexcept;
    
    // 缓冲区操作
    Result<size_t> available() const;
    VoidResult flushInput();
    VoidResult flushOutput();
    VoidResult flush();
    
    // 控制线
    VoidResult setDTR(bool state);
    VoidResult setRTS(bool state);
    
    // 状态
    std::string portName() const noexcept;
    PortStatistics statistics() const noexcept;
    void resetStatistics() noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace csp

#endif // CSERIAL_PORT_H
