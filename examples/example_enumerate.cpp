/**
 * @file example_enumerate.cpp
 * @brief 枚举串口示例
 *
 * 用法: example_enumerate [-v]
 * 选项: -v  显示详细信息（包括硬件ID）
 */

#include "SerialPort.h"
#include <iostream>
#include <iomanip>

int main(int argc, char* argv[]) {
    std::cout << "CSerialPort 串口枚举示例" << std::endl;
    std::cout << "版本: " << csp::SerialPort::version() << std::endl;
    std::cout << "========================" << std::endl;

    // 检查是否需要详细输出
    bool verbose = false;
    if (argc >= 2) {
        std::string arg1 = argv[1];
        if (arg1 == "-v" || arg1 == "--verbose") {
            verbose = true;
        } else if (arg1 == "-h" || arg1 == "--help") {
            std::cout << "用法: " << argv[0] << " [-v]" << std::endl;
            std::cout << "选项: -v  显示详细信息（包括硬件ID）" << std::endl;
            return 0;
        }
    }

    // 枚举所有串口
    std::cout << "\n正在枚举系统串口..." << std::endl;

    auto ports = csp::SerialPort::enumerate();

    if (ports.empty()) {
        std::cout << "未发现任何串口" << std::endl;
        return 0;
    }

    std::cout << "\n发现 " << ports.size() << " 个串口:\n" << std::endl;

    // 打印表头
    if (verbose) {
        std::cout << std::left
                  << std::setw(15) << "端口名称"
                  << std::setw(35) << "描述"
                  << std::setw(10) << "状态"
                  << "硬件ID"
                  << std::endl;
        std::cout << std::string(90, '-') << std::endl;
    } else {
        std::cout << std::left
                  << std::setw(15) << "端口名称"
                  << std::setw(35) << "描述"
                  << std::setw(10) << "状态"
                  << std::endl;
        std::cout << std::string(60, '-') << std::endl;
    }

    // 打印每个串口信息
    for (const auto& port : ports) {
        std::cout << std::left
                  << std::setw(15) << port.portName
                  << std::setw(35) << (port.description.empty() ? "-" : port.description)
                  << std::setw(10) << (port.isAvailable ? "可用" : "占用");

        if (verbose && !port.hardwareId.empty()) {
            std::cout << port.hardwareId;
        }

        std::cout << std::endl;
    }

    std::cout << std::endl;

    // 统计可用端口数量
    size_t availableCount = 0;
    for (const auto& port : ports) {
        if (port.isAvailable) {
            ++availableCount;
        }
    }
    std::cout << "可用端口: " << availableCount << " / " << ports.size() << std::endl;

    // 检查特定串口是否存在
    std::string testPort;
#if CSERIALPORT_PLATFORM_WINDOWS
    testPort = "COM1";
#else
    testPort = "/dev/ttyUSB0";
#endif

    std::cout << "\n检查串口 " << testPort << " 是否存在: ";
    if (csp::SerialPort::exists(testPort)) {
        std::cout << "是" << std::endl;
    } else {
        std::cout << "否" << std::endl;
    }

    return 0;
}
