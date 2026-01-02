/**
 * @file example_async.cpp
 * @brief 异步接收示例
 *
 * 用法: example_async [端口名] [波特率]
 * 示例: example_async COM1 115200
 *       example_async /dev/ttyUSB0 9600
 */

#include "SerialPort.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>

std::atomic<bool> running{true};

void printUsage(const char* programName) {
    std::cout << "用法: " << programName << " [端口名] [波特率]" << std::endl;
    std::cout << "示例: " << programName << " COM1 115200" << std::endl;
    std::cout << "      " << programName << " /dev/ttyUSB0 9600" << std::endl;
}

std::string getDefaultPortName() {
#if CSERIALPORT_PLATFORM_WINDOWS
    return "COM1";
#else
    return "/dev/ttyUSB0";
#endif
}

csp::BaudRate parseBaudRate(uint32_t baud) {
    switch (baud) {
        case 9600:   return csp::BaudRate::BR_9600;
        case 19200:  return csp::BaudRate::BR_19200;
        case 38400:  return csp::BaudRate::BR_38400;
        case 57600:  return csp::BaudRate::BR_57600;
        case 115200: return csp::BaudRate::BR_115200;
        case 230400: return csp::BaudRate::BR_230400;
        default:     return csp::BaudRate::BR_115200;
    }
}

int main(int argc, char* argv[]) {
    std::cout << "CSerialPort 异步接收示例" << std::endl;
    std::cout << "版本: " << csp::SerialPort::version() << std::endl;
    std::cout << "========================" << std::endl;

    // 解析命令行参数
    std::string portName = getDefaultPortName();
    uint32_t baudRate = 115200;

    if (argc >= 2) {
        std::string arg1 = argv[1];
        if (arg1 == "-h" || arg1 == "--help") {
            printUsage(argv[0]);
            return 0;
        }
        portName = arg1;
    }

    if (argc >= 3) {
        try {
            baudRate = static_cast<uint32_t>(std::stoul(argv[2]));
        } catch (...) {
            std::cerr << "无效的波特率: " << argv[2] << std::endl;
            return 1;
        }
    }

    // 如果没有指定端口，尝试自动选择第一个可用端口
    if (argc < 2) {
        auto ports = csp::SerialPort::enumerate();
        if (!ports.empty()) {
            for (const auto& p : ports) {
                if (p.isAvailable) {
                    portName = p.portName;
                    std::cout << "自动选择端口: " << portName << std::endl;
                    break;
                }
            }
        }
    }

    // 创建串口对象
    csp::SerialPort port;

    // 配置串口
    csp::SerialConfig config;
    config.baudRate = parseBaudRate(baudRate);
    config.dataBits = csp::DataBits::Eight;
    config.stopBits = csp::StopBits::One;
    config.parity = csp::Parity::None;

    std::cout << "尝试打开串口: " << portName << " @ " << baudRate << " bps" << std::endl;

    // 打开串口
    auto openResult = port.open(portName, config);

    if (!openResult) {
        std::cerr << "打开串口失败: " << openResult.errorMessage() << std::endl;
        return 1;
    }

    std::cout << "串口已打开!" << std::endl;

    // 设置数据接收回调
    port.setDataCallback([](const csp::Byte* data, size_t size) {
        std::cout << "[数据] 收到 " << size << " 字节: ";
        for (size_t i = 0; i < size; ++i) {
            if (data[i] >= 32 && data[i] < 127) {
                std::cout << static_cast<char>(data[i]);
            } else {
                std::cout << "[0x" << std::hex << static_cast<int>(data[i]) << std::dec << "]";
            }
        }
        std::cout << std::endl;
    });

    // 设置事件回调
    port.setEventCallback([](csp::EventType event) {
        switch (event) {
            case csp::EventType::DataReceived:
                // 数据接收事件（已在数据回调中处理）
                break;
            case csp::EventType::DataSent:
                std::cout << "[事件] 数据发送完成" << std::endl;
                break;
            case csp::EventType::Error:
                std::cout << "[事件] 发生错误" << std::endl;
                break;
            default:
                std::cout << "[事件] 未知事件: " << static_cast<int>(event) << std::endl;
                break;
        }
    });

    // 设置错误回调
    port.setErrorCallback([](csp::ErrorCode error, const std::string& message) {
        std::cerr << "[错误] " << message << " (错误码: " << static_cast<int>(error) << ")" << std::endl;
    });

    // 启动异步接收
    std::cout << "\n启动异步接收..." << std::endl;
    auto startResult = port.startAsyncReceive();

    if (!startResult) {
        std::cerr << "启动异步接收失败: " << startResult.errorMessage() << std::endl;
        return 1;
    }

    std::cout << "异步接收已启动!" << std::endl;
    std::cout << "命令:" << std::endl;
    std::cout << "  q        - 退出程序" << std::endl;
    std::cout << "  s        - 显示统计信息" << std::endl;
    std::cout << "  c        - 显示控制线状态" << std::endl;
    std::cout << "  b        - 发送 Break 信号" << std::endl;
    std::cout << "  其他内容 - 发送到串口" << std::endl;
    std::cout << std::endl;

    // 主循环
    std::string input;
    while (running) {
        std::getline(std::cin, input);

        if (input == "q" || input == "Q") {
            running = false;
            break;
        }

        if (input == "s" || input == "S") {
            auto stats = port.statistics();
            std::cout << "\n统计信息:" << std::endl;
            std::cout << "  发送字节数: " << stats.getBytesSent() << std::endl;
            std::cout << "  接收字节数: " << stats.getBytesReceived() << std::endl;
            std::cout << "  读取错误: " << stats.getReadErrors() << std::endl;
            std::cout << "  写入错误: " << stats.getWriteErrors() << std::endl;
            continue;
        }

        if (input == "c" || input == "C") {
            std::cout << "\n控制线状态:" << std::endl;
            auto cts = port.getCTS();
            auto dsr = port.getDSR();
            auto cd = port.getCD();
            auto ri = port.getRI();
            if (cts) std::cout << "  CTS: " << (cts.value() ? "高" : "低") << std::endl;
            if (dsr) std::cout << "  DSR: " << (dsr.value() ? "高" : "低") << std::endl;
            if (cd)  std::cout << "  CD:  " << (cd.value() ? "高" : "低") << std::endl;
            if (ri)  std::cout << "  RI:  " << (ri.value() ? "高" : "低") << std::endl;
            continue;
        }

        if (input == "b" || input == "B") {
            std::cout << "发送 Break 信号..." << std::endl;
            auto breakResult = port.sendBreak();
            if (breakResult) {
                std::cout << "Break 信号已发送" << std::endl;
            } else {
                std::cerr << "发送 Break 失败: " << breakResult.errorMessage() << std::endl;
            }
            continue;
        }

        if (!input.empty()) {
            // 发送输入的内容
            auto writeResult = port.writeLine(input);
            if (writeResult) {
                std::cout << "[发送] " << writeResult.value() << " 字节" << std::endl;
            } else {
                std::cerr << "[发送失败] " << writeResult.errorMessage() << std::endl;
            }
        }
    }

    // 停止异步接收
    std::cout << "\n停止异步接收..." << std::endl;
    port.stopAsyncReceive();

    // 打印统计信息
    auto stats = port.statistics();
    std::cout << "\n最终统计信息:" << std::endl;
    std::cout << "  发送字节数: " << stats.getBytesSent() << std::endl;
    std::cout << "  接收字节数: " << stats.getBytesReceived() << std::endl;

    // 关闭串口
    port.close();
    std::cout << "串口已关闭" << std::endl;

    return 0;
}
