/**
 * @file example_enumerate.cpp
 * @brief 枚举串口示例
 */

#include "SerialPort.h"
#include <iostream>
#include <iomanip>

int main() {
    std::cout << "CSerialPort 串口枚举示例" << std::endl;
    std::cout << "版本: " << csp::SerialPort::version() << std::endl;
    std::cout << "========================" << std::endl;
    
    // 枚举所有串口
    std::cout << "\n正在枚举系统串口..." << std::endl;
    
    auto ports = csp::SerialPort::enumerate();
    
    if (ports.empty()) {
        std::cout << "未发现任何串口" << std::endl;
        return 0;
    }
    
    std::cout << "\n发现 " << ports.size() << " 个串口:\n" << std::endl;
    
    // 打印表头
    std::cout << std::left 
              << std::setw(15) << "端口名称"
              << std::setw(30) << "描述"
              << std::setw(10) << "状态"
              << std::endl;
    std::cout << std::string(55, '-') << std::endl;
    
    // 打印每个串口信息
    for (const auto& port : ports) {
        std::cout << std::left
                  << std::setw(15) << port.portName
                  << std::setw(30) << (port.description.empty() ? "-" : port.description)
                  << std::setw(10) << (port.isAvailable ? "可用" : "占用")
                  << std::endl;
    }
    
    std::cout << std::endl;
    
    // 检查特定串口是否存在
    std::string testPort;
#if CSERIALPORT_PLATFORM_WINDOWS
    testPort = "COM1";
#else
    testPort = "/dev/ttyUSB0";
#endif
    
    std::cout << "检查串口 " << testPort << " 是否存在: ";
    if (csp::SerialPort::exists(testPort)) {
        std::cout << "是" << std::endl;
    } else {
        std::cout << "否" << std::endl;
    }
    
    return 0;
}