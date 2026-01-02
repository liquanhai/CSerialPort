/**
 * @file example_basic.cpp
 * @brief 基本使用示例
 *
 * 用法: example_basic [端口名] [波特率]
 * 示例: example_basic COM1 115200
 *       example_basic /dev/ttyUSB0 9600
 */

#include "SerialPort.h"
#include <iostream>
#include <string>

void printUsage(const char* programName) {
    std::cout << "用法: " << programName << " [端口名] [波特率]" << std::endl;
    std::cout << "示例: " << programName << " COM1 115200" << std::endl;
    std::cout << "      " << programName << " /dev/ttyUSB0 9600" << std::endl;
    std::cout << std::endl;
    std::cout << "如果不指定参数，将使用默认端口和波特率" << std::endl;
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
        case 110:    return csp::BaudRate::BR_110;
        case 300:    return csp::BaudRate::BR_300;
        case 600:    return csp::BaudRate::BR_600;
        case 1200:   return csp::BaudRate::BR_1200;
        case 2400:   return csp::BaudRate::BR_2400;
        case 4800:   return csp::BaudRate::BR_4800;
        case 9600:   return csp::BaudRate::BR_9600;
        case 14400:  return csp::BaudRate::BR_14400;
        case 19200:  return csp::BaudRate::BR_19200;
        case 38400:  return csp::BaudRate::BR_38400;
        case 57600:  return csp::BaudRate::BR_57600;
        case 115200: return csp::BaudRate::BR_115200;
        case 230400: return csp::BaudRate::BR_230400;
        case 460800: return csp::BaudRate::BR_460800;
        case 921600: return csp::BaudRate::BR_921600;
        default:     return csp::BaudRate::BR_9600;
    }
}

int main(int argc, char* argv[]) {
    std::cout << "CSerialPort 基本使用示例" << std::endl;
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

    // 配置串口参数
    csp::SerialConfig config;
    config.baudRate = parseBaudRate(baudRate);
    config.dataBits = csp::DataBits::Eight;
    config.stopBits = csp::StopBits::One;
    config.parity = csp::Parity::None;
    config.readTimeout = csp::Duration(2000);  // 2秒读取超时
    config.writeTimeout = csp::Duration(1000); // 1秒写入超时

    std::cout << "尝试打开串口: " << portName << std::endl;

    // 打开串口
    auto openResult = port.open(portName, config);

    if (!openResult) {
        std::cerr << "打开串口失败: " << openResult.errorMessage() << std::endl;
        std::cerr << "错误码: " << static_cast<int>(openResult.error()) << std::endl;
        return 1;
    }

    std::cout << "串口已打开!" << std::endl;
    std::cout << "当前配置:" << std::endl;
    std::cout << "  波特率: " << config.getBaudRateValue() << std::endl;
    std::cout << "  数据位: " << static_cast<int>(config.dataBits) << std::endl;
    std::cout << "  停止位: " << static_cast<int>(config.stopBits) << std::endl;
    std::cout << "  校验位: " << static_cast<int>(config.parity) << std::endl;

    // 显示控制线状态
    std::cout << "\n控制线状态:" << std::endl;
    auto cts = port.getCTS();
    auto dsr = port.getDSR();
    auto cd = port.getCD();
    auto ri = port.getRI();
    if (cts) std::cout << "  CTS: " << (cts.value() ? "高" : "低") << std::endl;
    if (dsr) std::cout << "  DSR: " << (dsr.value() ? "高" : "低") << std::endl;
    if (cd)  std::cout << "  CD:  " << (cd.value() ? "高" : "低") << std::endl;
    if (ri)  std::cout << "  RI:  " << (ri.value() ? "高" : "低") << std::endl;

    // 发送数据
    std::string message = "Hello, Serial Port!\r\n";
    std::cout << "\n发送数据: " << message;

    auto writeResult = port.write(message);

    if (writeResult) {
        std::cout << "成功发送 " << writeResult.value() << " 字节" << std::endl;
    } else {
        std::cerr << "发送失败: " << writeResult.errorMessage() << std::endl;
    }

    // 接收数据
    std::cout << "\n等待接收数据..." << std::endl;

    auto readResult = port.read(100);

    if (readResult) {
        auto& data = readResult.value();
        std::cout << "接收到 " << data.size() << " 字节: ";
        for (auto byte : data) {
            if (byte >= 32 && byte < 127) {
                std::cout << static_cast<char>(byte);
            } else {
                std::cout << "[0x" << std::hex << static_cast<int>(byte) << std::dec << "]";
            }
        }
        std::cout << std::endl;
    } else {
        if (readResult.error() == csp::ErrorCode::Timeout) {
            std::cout << "读取超时，没有收到数据" << std::endl;
        } else {
            std::cerr << "读取失败: " << readResult.errorMessage() << std::endl;
        }
    }

    // 获取统计信息
    auto stats = port.statistics();
    std::cout << "\n统计信息:" << std::endl;
    std::cout << "  发送字节数: " << stats.getBytesSent() << std::endl;
    std::cout << "  接收字节数: " << stats.getBytesReceived() << std::endl;
    std::cout << "  读取错误: " << stats.getReadErrors() << std::endl;
    std::cout << "  写入错误: " << stats.getWriteErrors() << std::endl;

    // 关闭串口（也可以让析构函数自动关闭）
    port.close();
    std::cout << "\n串口已关闭" << std::endl;

    return 0;
}
