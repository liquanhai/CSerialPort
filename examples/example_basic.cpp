/**
 * @file example_basic.cpp
 * @brief 基本使用示例
 */

#include "SerialPort.h"
#include <iostream>
#include <string>

int main() {
    std::cout << "CSerialPort 基本使用示例" << std::endl;
    std::cout << "版本: " << csp::SerialPort::version() << std::endl;
    std::cout << "========================" << std::endl;
    
    // 创建串口对象
    csp::SerialPort port;
    
    // 配置串口参数
    csp::SerialConfig config = csp::SerialConfig::config_115200_8N1();
    config.readTimeout = csp::Duration(2000);  // 2秒读取超时
    config.writeTimeout = csp::Duration(1000); // 1秒写入超时
    
    // 获取串口名称
    std::string portName;
#if CSERIALPORT_PLATFORM_WINDOWS
    portName = "COM1";
#else
    portName = "/dev/ttyUSB0";
#endif
    
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
    std::cout << "  发送字节数: " << stats.bytesSent << std::endl;
    std::cout << "  接收字节数: " << stats.bytesReceived << std::endl;
    std::cout << "  读取错误: " << stats.readErrors << std::endl;
    std::cout << "  写入错误: " << stats.writeErrors << std::endl;
    
    // 关闭串口（也可以让析构函数自动关闭）
    port.close();
    std::cout << "\n串口已关闭" << std::endl;
    
    return 0;
}