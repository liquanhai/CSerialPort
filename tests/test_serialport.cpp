/**
 * @file test_serialport.cpp
 * @brief CSerialPort 单元测试
 *
 * 使用简单的测试框架，不依赖外部库
 */

#include "SerialPort.h"
#include <iostream>
#include <cassert>
#include <sstream>

// 简单的测试框架
#define TEST_CASE(name) void test_##name()
#define RUN_TEST(name) do { \
    std::cout << "Running " << #name << "... "; \
    try { \
        test_##name(); \
        std::cout << "PASSED" << std::endl; \
        ++passed; \
    } catch (const std::exception& e) { \
        std::cout << "FAILED: " << e.what() << std::endl; \
        ++failed; \
    } catch (...) { \
        std::cout << "FAILED: Unknown exception" << std::endl; \
        ++failed; \
    } \
} while(0)

#define ASSERT_TRUE(expr) do { \
    if (!(expr)) { \
        throw std::runtime_error("Assertion failed: " #expr); \
    } \
} while(0)

#define ASSERT_FALSE(expr) ASSERT_TRUE(!(expr))

#define ASSERT_EQ(a, b) do { \
    if ((a) != (b)) { \
        std::ostringstream oss; \
        oss << "Assertion failed: " << #a << " == " << #b; \
        throw std::runtime_error(oss.str()); \
    } \
} while(0)

#define ASSERT_NE(a, b) do { \
    if ((a) == (b)) { \
        std::ostringstream oss; \
        oss << "Assertion failed: " << #a << " != " << #b; \
        throw std::runtime_error(oss.str()); \
    } \
} while(0)

// ============================================================================
// Result<T> 类型测试
// ============================================================================

TEST_CASE(result_success_value) {
    csp::Result<int> result(42);
    ASSERT_TRUE(result.hasValue());
    ASSERT_TRUE(static_cast<bool>(result));
    ASSERT_EQ(result.value(), 42);
    ASSERT_EQ(*result, 42);
}

TEST_CASE(result_error) {
    csp::Result<int> result(csp::ErrorCode::InvalidParameter, "Test error");
    ASSERT_FALSE(result.hasValue());
    ASSERT_FALSE(static_cast<bool>(result));
    ASSERT_EQ(result.error(), csp::ErrorCode::InvalidParameter);
    ASSERT_EQ(result.errorMessage(), "Test error");
}

TEST_CASE(result_value_or) {
    csp::Result<int> success(42);
    csp::Result<int> error(csp::ErrorCode::Unknown, "Error");

    ASSERT_EQ(success.valueOr(0), 42);
    ASSERT_EQ(error.valueOr(100), 100);
}

TEST_CASE(result_void_success) {
    csp::VoidResult result;
    ASSERT_TRUE(result.hasValue());
    ASSERT_TRUE(static_cast<bool>(result));
}

TEST_CASE(result_void_error) {
    csp::VoidResult result(csp::ErrorCode::NotOpen, "Port not open");
    ASSERT_FALSE(result.hasValue());
    ASSERT_EQ(result.error(), csp::ErrorCode::NotOpen);
}

TEST_CASE(result_move) {
    csp::Result<std::string> result1("Hello");
    csp::Result<std::string> result2 = std::move(result1);
    ASSERT_TRUE(result2.hasValue());
    ASSERT_EQ(result2.value(), "Hello");
}

// ============================================================================
// SerialConfig 测试
// ============================================================================

TEST_CASE(config_default) {
    csp::SerialConfig config;
    ASSERT_EQ(config.baudRate, csp::BaudRate::BR_9600);
    ASSERT_EQ(config.dataBits, csp::DataBits::Eight);
    ASSERT_EQ(config.stopBits, csp::StopBits::One);
    ASSERT_EQ(config.parity, csp::Parity::None);
    ASSERT_EQ(config.flowControl, csp::FlowControl::None);
}

TEST_CASE(config_9600_8n1) {
    auto config = csp::SerialConfig::config_9600_8N1();
    ASSERT_EQ(config.getBaudRateValue(), 9600u);
    ASSERT_EQ(config.dataBits, csp::DataBits::Eight);
    ASSERT_EQ(config.stopBits, csp::StopBits::One);
    ASSERT_EQ(config.parity, csp::Parity::None);
}

TEST_CASE(config_115200_8n1) {
    auto config = csp::SerialConfig::config_115200_8N1();
    ASSERT_EQ(config.getBaudRateValue(), 115200u);
    ASSERT_EQ(config.dataBits, csp::DataBits::Eight);
    ASSERT_EQ(config.stopBits, csp::StopBits::One);
    ASSERT_EQ(config.parity, csp::Parity::None);
}

TEST_CASE(config_custom_baudrate) {
    csp::SerialConfig config;
    config.baudRate = csp::BaudRate::Custom;
    config.customBaudRate = 250000;
    ASSERT_EQ(config.getBaudRateValue(), 250000u);
}

// ============================================================================
// PortInfo 测试
// ============================================================================

TEST_CASE(portinfo_display_name) {
    csp::PortInfo info;
    info.portName = "COM1";
    info.description = "";
    ASSERT_EQ(info.displayName(), "COM1");

    info.description = "USB Serial Port";
    ASSERT_EQ(info.displayName(), "COM1 - USB Serial Port");
}

// ============================================================================
// PortStatistics 测试
// ============================================================================

TEST_CASE(statistics_default) {
    csp::PortStatistics stats;
    ASSERT_EQ(stats.getBytesReceived(), 0u);
    ASSERT_EQ(stats.getBytesSent(), 0u);
    ASSERT_EQ(stats.getReadErrors(), 0u);
    ASSERT_EQ(stats.getWriteErrors(), 0u);
}

TEST_CASE(statistics_atomic_operations) {
    csp::PortStatistics stats;
    stats.bytesReceived.fetch_add(100, std::memory_order_relaxed);
    stats.bytesSent.fetch_add(50, std::memory_order_relaxed);
    stats.readErrors.fetch_add(1, std::memory_order_relaxed);
    stats.writeErrors.fetch_add(2, std::memory_order_relaxed);

    ASSERT_EQ(stats.getBytesReceived(), 100u);
    ASSERT_EQ(stats.getBytesSent(), 50u);
    ASSERT_EQ(stats.getReadErrors(), 1u);
    ASSERT_EQ(stats.getWriteErrors(), 2u);
}

TEST_CASE(statistics_reset) {
    csp::PortStatistics stats;
    stats.bytesReceived.store(100, std::memory_order_relaxed);
    stats.bytesSent.store(50, std::memory_order_relaxed);
    stats.reset();

    ASSERT_EQ(stats.getBytesReceived(), 0u);
    ASSERT_EQ(stats.getBytesSent(), 0u);
}

TEST_CASE(statistics_copy) {
    csp::PortStatistics stats1;
    stats1.bytesReceived.store(100, std::memory_order_relaxed);
    stats1.bytesSent.store(50, std::memory_order_relaxed);

    csp::PortStatistics stats2(stats1);
    ASSERT_EQ(stats2.getBytesReceived(), 100u);
    ASSERT_EQ(stats2.getBytesSent(), 50u);
}

TEST_CASE(statistics_timestamps) {
    csp::PortStatistics stats;

    // 初始状态：运行时长应该很小
    auto uptime = stats.getUptime();
    ASSERT_TRUE(uptime.count() >= 0);

    // 初始状态：最后活动时间偏移应该为0
    auto lastActivityOffset = stats.getLastActivityOffset();
    ASSERT_EQ(lastActivityOffset.count(), 0);

    // 初始状态：自最后活动以来的时间应该为0（因为尚无活动）
    auto timeSinceLastActivity = stats.getTimeSinceLastActivity();
    ASSERT_EQ(timeSinceLastActivity.count(), 0);

    // 更新最后活动时间
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    stats.updateLastActivity();

    // 现在最后活动时间偏移应该大于0
    lastActivityOffset = stats.getLastActivityOffset();
    ASSERT_TRUE(lastActivityOffset.count() > 0);

    // 自最后活动以来的时间应该很小
    timeSinceLastActivity = stats.getTimeSinceLastActivity();
    ASSERT_TRUE(timeSinceLastActivity.count() >= 0);
}

TEST_CASE(statistics_reset_timestamps) {
    csp::PortStatistics stats;

    // 更新活动时间
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    stats.updateLastActivity();

    // 重置
    stats.reset();

    // 重置后最后活动时间偏移应该为0
    auto lastActivityOffset = stats.getLastActivityOffset();
    ASSERT_EQ(lastActivityOffset.count(), 0);
}

// ============================================================================
// SerialError 测试
// ============================================================================

TEST_CASE(serial_error_code) {
    csp::SerialError error(csp::ErrorCode::NotOpen, "Port not open");
    ASSERT_EQ(error.code(), csp::ErrorCode::NotOpen);
    ASSERT_EQ(std::string(error.what()), "Port not open");
}

TEST_CASE(serial_error_code_to_string) {
    ASSERT_EQ(csp::SerialError::errorCodeToString(csp::ErrorCode::Success), "Success");
    ASSERT_EQ(csp::SerialError::errorCodeToString(csp::ErrorCode::NotOpen), "Port not open");
    ASSERT_EQ(csp::SerialError::errorCodeToString(csp::ErrorCode::Timeout), "Operation timeout");
    ASSERT_EQ(csp::SerialError::errorCodeToString(csp::ErrorCode::PortNotFound), "Port not found");
}

// ============================================================================
// SerialPort 基本测试
// ============================================================================

TEST_CASE(serialport_default_constructor) {
    csp::SerialPort port;
    ASSERT_FALSE(port.isOpen());
    ASSERT_FALSE(port.isAsyncReceiving());
}

TEST_CASE(serialport_version) {
    std::string version = csp::SerialPort::version();
    ASSERT_FALSE(version.empty());
    ASSERT_EQ(version, "3.0.0");
}

TEST_CASE(serialport_enumerate) {
    // 枚举不应该抛出异常
    auto ports = csp::SerialPort::enumerate();
    // 结果可能为空（没有串口），但不应该崩溃
    (void)ports;
}

TEST_CASE(serialport_open_invalid_port) {
    csp::SerialPort port;
    auto result = port.open("INVALID_PORT_NAME_12345");
    ASSERT_FALSE(result.hasValue());
    ASSERT_FALSE(port.isOpen());
}

TEST_CASE(serialport_close_not_open) {
    csp::SerialPort port;
    auto result = port.close();
    // 关闭未打开的端口应该成功（幂等操作）
    ASSERT_TRUE(result.hasValue());
}

TEST_CASE(serialport_read_not_open) {
    csp::SerialPort port;
    auto result = port.read(100);
    ASSERT_FALSE(result.hasValue());
    ASSERT_EQ(result.error(), csp::ErrorCode::NotOpen);
}

TEST_CASE(serialport_write_not_open) {
    csp::SerialPort port;
    auto result = port.write("test");
    ASSERT_FALSE(result.hasValue());
    ASSERT_EQ(result.error(), csp::ErrorCode::NotOpen);
}

TEST_CASE(serialport_move) {
    csp::SerialPort port1;
    csp::SerialPort port2 = std::move(port1);
    ASSERT_FALSE(port2.isOpen());
}

TEST_CASE(serialport_config_not_open) {
    csp::SerialPort port;
    auto result = port.setConfig(csp::SerialConfig::config_115200_8N1());
    ASSERT_FALSE(result.hasValue());
    ASSERT_EQ(result.error(), csp::ErrorCode::NotOpen);
}

TEST_CASE(serialport_async_receive_not_open) {
    csp::SerialPort port;
    auto result = port.startAsyncReceive();
    ASSERT_FALSE(result.hasValue());
    ASSERT_EQ(result.error(), csp::ErrorCode::NotOpen);
}

TEST_CASE(serialport_flush_not_open) {
    csp::SerialPort port;
    auto result = port.flush();
    ASSERT_FALSE(result.hasValue());
    ASSERT_EQ(result.error(), csp::ErrorCode::NotOpen);
}

TEST_CASE(serialport_control_lines_not_open) {
    csp::SerialPort port;

    auto dtr = port.setDTR(true);
    ASSERT_FALSE(dtr.hasValue());
    ASSERT_EQ(dtr.error(), csp::ErrorCode::NotOpen);

    auto rts = port.setRTS(true);
    ASSERT_FALSE(rts.hasValue());
    ASSERT_EQ(rts.error(), csp::ErrorCode::NotOpen);

    auto cts = port.getCTS();
    ASSERT_FALSE(cts.hasValue());
    ASSERT_EQ(cts.error(), csp::ErrorCode::NotOpen);

    auto dsr = port.getDSR();
    ASSERT_FALSE(dsr.hasValue());
    ASSERT_EQ(dsr.error(), csp::ErrorCode::NotOpen);
}

TEST_CASE(serialport_send_break_not_open) {
    csp::SerialPort port;
    auto result = port.sendBreak();
    ASSERT_FALSE(result.hasValue());
    ASSERT_EQ(result.error(), csp::ErrorCode::NotOpen);
}

// ============================================================================
// 主函数
// ============================================================================

int main() {
    std::cout << "CSerialPort Unit Tests" << std::endl;
    std::cout << "======================" << std::endl;
    std::cout << "Version: " << csp::SerialPort::version() << std::endl;
    std::cout << std::endl;

    int passed = 0;
    int failed = 0;

    // Result<T> 测试
    std::cout << "\n--- Result<T> Tests ---" << std::endl;
    RUN_TEST(result_success_value);
    RUN_TEST(result_error);
    RUN_TEST(result_value_or);
    RUN_TEST(result_void_success);
    RUN_TEST(result_void_error);
    RUN_TEST(result_move);

    // SerialConfig 测试
    std::cout << "\n--- SerialConfig Tests ---" << std::endl;
    RUN_TEST(config_default);
    RUN_TEST(config_9600_8n1);
    RUN_TEST(config_115200_8n1);
    RUN_TEST(config_custom_baudrate);

    // PortInfo 测试
    std::cout << "\n--- PortInfo Tests ---" << std::endl;
    RUN_TEST(portinfo_display_name);

    // PortStatistics 测试
    std::cout << "\n--- PortStatistics Tests ---" << std::endl;
    RUN_TEST(statistics_default);
    RUN_TEST(statistics_atomic_operations);
    RUN_TEST(statistics_reset);
    RUN_TEST(statistics_copy);
    RUN_TEST(statistics_timestamps);
    RUN_TEST(statistics_reset_timestamps);

    // SerialError 测试
    std::cout << "\n--- SerialError Tests ---" << std::endl;
    RUN_TEST(serial_error_code);
    RUN_TEST(serial_error_code_to_string);

    // SerialPort 测试
    std::cout << "\n--- SerialPort Tests ---" << std::endl;
    RUN_TEST(serialport_default_constructor);
    RUN_TEST(serialport_version);
    RUN_TEST(serialport_enumerate);
    RUN_TEST(serialport_open_invalid_port);
    RUN_TEST(serialport_close_not_open);
    RUN_TEST(serialport_read_not_open);
    RUN_TEST(serialport_write_not_open);
    RUN_TEST(serialport_move);
    RUN_TEST(serialport_config_not_open);
    RUN_TEST(serialport_async_receive_not_open);
    RUN_TEST(serialport_flush_not_open);
    RUN_TEST(serialport_control_lines_not_open);
    RUN_TEST(serialport_send_break_not_open);

    // 总结
    std::cout << "\n======================" << std::endl;
    std::cout << "Total: " << (passed + failed) << " tests" << std::endl;
    std::cout << "Passed: " << passed << std::endl;
    std::cout << "Failed: " << failed << std::endl;

    return failed > 0 ? 1 : 0;
}
