/**
 * @file example_async.cpp
 * @brief 异步接收示例
 */

#include "SerialPort.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>

std::atomic<bool> running{true};

int main() {
    std::cout << "CSerialPort 异步接收示例" << std::endl;
    std::cout << "版本: " << csp::SerialPort::version() << std::endl;
    std::cout << "========================" << std::endl;
    
    // 创建串口对象
    csp::SerialPort port;
    
    // 获取串口名称
    std::string portName;
#if CSERIALPORT_PLATFORM_WINDOWS
    portName = "COM1";
#else
    portName = "/dev/ttyUSB0";
#endif
    
    std::cout << "尝试打开串口: " << portName << std::endl;
    
    // 打开串口
    auto openResult = port.open(portName, csp::SerialConfig::config_115200_8N1());
    
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
    std::cout << "输入 'q' 并按 Enter 退出，或输入其他内容发送到串口" << std::endl;
    std::cout << std::endl;
    
    // 主循环
    std::string input;
    while (running) {
        std::getline(std::cin, input);
        
        if (input == "q" || input == "Q") {
            running = false;
            break;
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
    std::cout << "\n统计信息:" << std::endl;
    std::cout << "  发送字节数: " << stats.bytesSent << std::endl;
    std::cout << "  接收字节数: " << stats.bytesReceived << std::endl;
    
    // 关闭串口
    port.close();
    std::cout << "串口已关闭" << std::endl;
    
    return 0;
}