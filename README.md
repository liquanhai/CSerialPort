# CSerialPort - 现代C++跨平台串口通信库

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://isocpp.org/std/the-standard)
[![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20Linux-green.svg)]()

## 简介

CSerialPort 是一个使用现代 C++17 编写的跨平台串口通信库，支持 Windows 和 Linux 操作系统。本库提供了简洁易用的 API，支持同步和异步操作，以及回调机制。

## 特性

- 🚀 **现代 C++17** - 使用最新的 C++ 特性，如 `std::optional`、`std::variant`、智能指针等
- 🌍 **跨平台支持** - 同时支持 Windows 和 Linux
- 🔒 **线程安全** - 内置互斥锁保护，支持多线程环境
- ⚡ **异步操作** - 支持 `std::future` 异步读写
- 📞 **回调机制** - 支持数据接收、事件和错误回调
- 🛡️ **RAII 资源管理** - 自动管理串口资源，防止泄漏
- 📊 **统计信息** - 内置收发字节数和错误统计
- 🔧 **灵活配置** - 支持各种波特率、数据位、停止位、校验位和流控制

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| 3.0.0 | 2026-01-01 | 使用现代C++17重写，支持跨平台 |
| 2.0.0 | 2014-01-10 | 添加串口枚举功能 |
| 1.0.0 | 1997-09-15 | 初始版本 |

## 快速开始

### 编译要求

- C++17 兼容的编译器
  - GCC 7+ 
  - Clang 5+
  - MSVC 2017+
- Windows: Windows SDK
- Linux: 无额外依赖

### 基本使用

```cpp
#include "SerialPort.h"
#include <iostream>

int main() {
    // 创建串口对象
    csp::SerialPort port;
    
    // 打开串口（Windows: "COM1", Linux: "/dev/ttyUSB0"）
    auto result = port.open("COM1", csp::SerialConfig::config_115200_8N1());
    
    if (!result) {
        std::cerr << "打开串口失败: " << result.errorMessage() << std::endl;
        return 1;
    }
    
    // 发送数据
    std::string message = "Hello, Serial Port!";
    auto writeResult = port.write(message);
    
    if (writeResult) {
        std::cout << "发送了 " << writeResult.value() << " 字节" << std::endl;
    }
    
    // 接收数据
    auto readResult = port.read(100);
    
    if (readResult) {
        auto& data = readResult.value();
        std::cout << "接收了 " << data.size() << " 字节" << std::endl;
    }
    
    // 串口会在析构时自动关闭
    return 0;
}
```

## API 参考

### 命名空间

所有类型和函数都在 `csp` 命名空间中。

### 枚举类型

#### BaudRate - 波特率

```cpp
enum class BaudRate : uint32_t {
    BR_9600   = 9600,
    BR_19200  = 19200,
    BR_38400  = 38400,
    BR_57600  = 57600,
    BR_115200 = 115200,
    // ... 更多选项
    Custom    = 0  // 自定义波特率
};
```

#### DataBits - 数据位

```cpp
enum class DataBits : uint8_t {
    Five  = 5,
    Six   = 6,
    Seven = 7,
    Eight = 8
};
```

#### StopBits - 停止位

```cpp
enum class StopBits : uint8_t {
    One     = 1,
    OneHalf = 2,  // 仅Windows支持
    Two     = 3
};
```

#### Parity - 校验位

```cpp
enum class Parity : uint8_t {
    None  = 0,
    Odd   = 1,
    Even  = 2,
    Mark  = 3,
    Space = 4
};
```

#### FlowControl - 流控制

```cpp
enum class FlowControl : uint8_t {
    None     = 0,
    Hardware = 1,  // RTS/CTS
    Software = 2   // XON/XOFF
};
```

### SerialConfig - 配置结构体

```cpp
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
    
    // 便捷工厂方法
    static SerialConfig defaultConfig();
    static SerialConfig config_9600_8N1();
    static SerialConfig config_115200_8N1();
};
```

### SerialPort - 主类

#### 构造函数

```cpp
SerialPort();  // 默认构造
SerialPort(const std::string& portName, const SerialConfig& config);  // 构造并打开
```

#### 静态方法

```cpp
// 枚举系统中的所有串口
static std::vector<PortInfo> enumerate();

// 检查串口是否存在
static bool exists(const std::string& portName);

// 获取库版本
static std::string version();
```

#### 打开/关闭

```cpp
VoidResult open(const std::string& portName, const SerialConfig& config);
VoidResult close();
bool isOpen() const;
```

#### 配置

```cpp
SerialConfig config() const;
VoidResult setConfig(const SerialConfig& config);
VoidResult setBaudRate(BaudRate baudRate);
VoidResult setDataBits(DataBits dataBits);
VoidResult setStopBits(StopBits stopBits);
VoidResult setParity(Parity parity);
VoidResult setFlowControl(FlowControl flowControl);
VoidResult setReadTimeout(Duration timeout);
VoidResult setWriteTimeout(Duration timeout);
```

#### 同步读取

```cpp
// 读取最多 maxBytes 字节
Result<ByteBuffer> read(size_t maxBytes, std::optional<Duration> timeout = std::nullopt);

// 精确读取 exactBytes 字节
Result<ByteBuffer> readExact(size_t exactBytes, std::optional<Duration> timeout = std::nullopt);

// 读取直到遇到分隔符
Result<ByteBuffer> readUntil(Byte delimiter, size_t maxBytes = 4096, 
                              std::optional<Duration> timeout = std::nullopt);

// 读取一行
Result<std::string> readLine(size_t maxBytes = 4096, 
                              std::optional<Duration> timeout = std::nullopt);
```

#### 同步写入

```cpp
Result<size_t> write(const Byte* data, size_t size, std::optional<Duration> timeout = std::nullopt);
Result<size_t> write(const ByteBuffer& data, std::optional<Duration> timeout = std::nullopt);
Result<size_t> write(const std::string& str, std::optional<Duration> timeout = std::nullopt);
Result<size_t> writeLine(const std::string& line, std::optional<Duration> timeout = std::nullopt);
```

#### 异步操作

```cpp
std::future<Result<ByteBuffer>> readAsync(size_t maxBytes);
std::future<Result<size_t>> writeAsync(ByteBuffer data);
std::future<Result<size_t>> writeAsync(std::string str);
```

#### 回调模式

```cpp
void setDataCallback(DataCallback callback);
void setEventCallback(EventCallback callback);
void setErrorCallback(ErrorCallback callback);
VoidResult startAsyncReceive();
void stopAsyncReceive();
bool isAsyncReceiving() const;
```

#### 缓冲区操作

```cpp
Result<size_t> available() const;  // 获取可读字节数
VoidResult flushInput();           // 清空接收缓冲区
VoidResult flushOutput();          // 清空发送缓冲区
VoidResult flush();                // 清空所有缓冲区
```

#### 控制线

```cpp
VoidResult setDTR(bool state);
VoidResult setRTS(bool state);
```

#### 状态

```cpp
std::string portName() const;
PortStatistics statistics() const;
void resetStatistics();
```

## 使用示例

### 示例1：枚举串口

```cpp
#include "SerialPort.h"
#include <iostream>

int main() {
    auto ports = csp::SerialPort::enumerate();
    
    std::cout << "发现 " << ports.size() << " 个串口:" << std::endl;
    
    for (const auto& port : ports) {
        std::cout << "  " << port.portName;
        if (!port.description.empty()) {
            std::cout << " - " << port.description;
        }
        std::cout << (port.isAvailable ? " (可用)" : " (占用)") << std::endl;
    }
    
    return 0;
}
```

### 示例2：异步接收

```cpp
#include "SerialPort.h"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    csp::SerialPort port;
    
    if (!port.open("COM1", csp::SerialConfig::config_115200_8N1())) {
        std::cerr << "打开串口失败" << std::endl;
        return 1;
    }
    
    // 设置数据接收回调
    port.setDataCallback([](const csp::Byte* data, size_t size) {
        std::cout << "收到 " << size << " 字节: ";
        for (size_t i = 0; i < size; ++i) {
            std::cout << std::hex << static_cast<int>(data[i]) << " ";
        }
        std::cout << std::endl;
    });
    
    // 设置错误回调
    port.setErrorCallback([](csp::ErrorCode error, const std::string& message) {
        std::cerr << "错误: " << message << std::endl;
    });
    
    // 启动异步接收
    port.startAsyncReceive();
    
    // 主线程等待
    std::cout << "按 Enter 键退出..." << std::endl;
    std::cin.get();
    
    // 停止异步接收
    port.stopAsyncReceive();
    
    return 0;
}
```

### 示例3：使用 Future 异步操作

```cpp
#include "SerialPort.h"
#include <iostream>
#include <future>

int main() {
    csp::SerialPort port;
    
    if (!port.open("COM1", csp::SerialConfig::config_115200_8N1())) {
        return 1;
    }
    
    // 异步发送
    auto writeFuture = port.writeAsync("Hello, World!");
    
    // 异步接收
    auto readFuture = port.readAsync(100);
    
    // 等待发送完成
    auto writeResult = writeFuture.get();
    if (writeResult) {
        std::cout << "发送了 " << writeResult.value() << " 字节" << std::endl;
    }
    
    // 等待接收完成
    auto readResult = readFuture.get();
    if (readResult) {
        std::cout << "接收了 " << readResult.value().size() << " 字节" << std::endl;
    }
    
    return 0;
}
```

### 示例4：自定义配置

```cpp
#include "SerialPort.h"

int main() {
    csp::SerialPort port;
    
    // 创建自定义配置
    csp::SerialConfig config;
    config.baudRate = csp::BaudRate::BR_38400;
    config.dataBits = csp::DataBits::Seven;
    config.stopBits = csp::StopBits::Two;
    config.parity = csp::Parity::Even;
    config.flowControl = csp::FlowControl::Hardware;
    config.readTimeout = csp::Duration(2000);  // 2秒超时
    config.writeTimeout = csp::Duration(2000);
    
    if (!port.open("COM1", config)) {
        return 1;
    }
    
    // 动态修改配置
    port.setBaudRate(csp::BaudRate::BR_115200);
    
    return 0;
}
```

## 错误处理

本库使用 `Result<T>` 类型进行错误处理，类似于 Rust 的 `Result` 或 C++23 的 `std::expected`。

```cpp
auto result = port.read(100);

if (result) {
    // 成功
    auto& data = result.value();
    // 或使用 *result
} else {
    // 失败
    std::cerr << "错误码: " << static_cast<int>(result.error()) << std::endl;
    std::cerr << "错误信息: " << result.errorMessage() << std::endl;
}

// 使用 valueOr 提供默认值
auto data = result.valueOr(csp::ByteBuffer{});
```

### 错误码

| 错误码 | 说明 |
|--------|------|
| Success | 成功 |
| InvalidParameter | 无效参数 |
| NotOpen | 串口未打开 |
| AlreadyOpen | 串口已打开 |
| Timeout | 操作超时 |
| OpenFailed | 打开失败 |
| PortNotFound | 串口不存在 |
| PermissionDenied | 权限不足 |
| PortBusy | 串口被占用 |
| ConfigFailed | 配置失败 |
| ReadFailed | 读取失败 |
| WriteFailed | 写入失败 |

## 平台特定说明

### Windows

- 串口名称格式：`COM1`, `COM2`, ... `COM256`
- 支持 1.5 停止位
- 使用 Overlapped I/O 实现异步操作

### Linux

- 串口名称格式：`/dev/ttyS0`, `/dev/ttyUSB0`, `/dev/ttyACM0` 等
- 不支持 1.5 停止位
- 使用 select() 实现超时控制

## 线程安全

- 所有公共方法都是线程安全的
- 可以在多个线程中同时调用读写方法
- 回调函数在内部线程中执行，请注意同步

## 许可证

本项目采用 MIT 许可证。详见 [LICENSE](LICENSE) 文件。

## 贡献

欢迎提交 Issue 和 Pull Request！

## 联系方式

- 作者：CSerialPort Team
- 博客：https://blog.csdn.net/liquanhai
