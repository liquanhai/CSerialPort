# ============================================================================
# CSerialPort - 现代C++跨平台串口通信库
# ============================================================================
cmake_minimum_required(VERSION 3.14)

project(CSerialPort
    VERSION 3.0.0
    DESCRIPTION "Modern C++ Cross-Platform Serial Port Library"
    LANGUAGES CXX
)

# ============================================================================
# 编译选项
# ============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 导出编译命令（用于IDE和代码分析工具）
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ============================================================================
# 选项
# ============================================================================
option(CSERIALPORT_BUILD_SHARED "Build shared library" ON)
option(CSERIALPORT_BUILD_STATIC "Build static library" ON)
option(CSERIALPORT_BUILD_EXAMPLES "Build examples" ON)
option(CSERIALPORT_BUILD_TESTS "Build tests" OFF)

# ============================================================================
# 平台检测
# ============================================================================
if(WIN32)
    set(CSERIALPORT_PLATFORM "Windows")
    add_definitions(-DCSERIALPORT_PLATFORM_WINDOWS=1)
    add_definitions(-DCSERIALPORT_PLATFORM_LINUX=0)
elseif(UNIX AND NOT APPLE)
    set(CSERIALPORT_PLATFORM "Linux")
    add_definitions(-DCSERIALPORT_PLATFORM_WINDOWS=0)
    add_definitions(-DCSERIALPORT_PLATFORM_LINUX=1)
elseif(APPLE)
    set(CSERIALPORT_PLATFORM "macOS")
    add_definitions(-DCSERIALPORT_PLATFORM_WINDOWS=0)
    add_definitions(-DCSERIALPORT_PLATFORM_LINUX=1)
else()
    message(FATAL_ERROR "Unsupported platform")
endif()

message(STATUS "CSerialPort version: ${PROJECT_VERSION}")
message(STATUS "Platform: ${CSERIALPORT_PLATFORM}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# ============================================================================
# 源文件
# ============================================================================
set(CSERIALPORT_SOURCES
    SerialPort.cpp
)

set(CSERIALPORT_HEADERS
    SerialPort.h
)

# ============================================================================
# 编译器警告
# ============================================================================
if(MSVC)
    add_compile_options(/W4 /WX-)
    add_compile_options(/utf-8)
    # 禁用一些不必要的警告
    add_compile_options(/wd4251)  # DLL导出警告
    add_compile_options(/wd4275)  # 非DLL接口类警告
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
    add_compile_options(-Wno-unused-parameter)
endif()

# ============================================================================
# 静态库
# ============================================================================
if(CSERIALPORT_BUILD_STATIC)
    add_library(cserialport_static STATIC ${CSERIALPORT_SOURCES} ${CSERIALPORT_HEADERS})
    
    target_include_directories(cserialport_static
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
            $<INSTALL_INTERFACE:include>
    )
    
    # 平台特定链接库
    if(WIN32)
        target_link_libraries(cserialport_static PUBLIC setupapi)
    elseif(UNIX)
        target_link_libraries(cserialport_static PUBLIC pthread)
    endif()
    
    set_target_properties(cserialport_static PROPERTIES
        OUTPUT_NAME cserialport
        ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
    )
    
    # 别名
    add_library(CSerialPort::static ALIAS cserialport_static)
endif()

# ============================================================================
# 动态库
# ============================================================================
if(CSERIALPORT_BUILD_SHARED)
    add_library(cserialport_shared SHARED ${CSERIALPORT_SOURCES} ${CSERIALPORT_HEADERS})
    
    target_include_directories(cserialport_shared
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
            $<INSTALL_INTERFACE:include>
    )
    
    # 平台特定链接库
    if(WIN32)
        target_link_libraries(cserialport_shared PUBLIC setupapi)
        target_compile_definitions(cserialport_shared PRIVATE CSERIALPORT_EXPORTS)
    elseif(UNIX)
        target_link_libraries(cserialport_shared PUBLIC pthread)
    endif()
    
    set_target_properties(cserialport_shared PROPERTIES
        OUTPUT_NAME cserialport
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
        VERSION ${PROJECT_VERSION}
        SOVERSION ${PROJECT_VERSION_MAJOR}
    )
    
    # 别名
    add_library(CSerialPort::shared ALIAS cserialport_shared)
endif()

# ============================================================================
# 默认库别名
# ============================================================================
if(CSERIALPORT_BUILD_STATIC)
    add_library(CSerialPort::CSerialPort ALIAS cserialport_static)
elseif(CSERIALPORT_BUILD_SHARED)
    add_library(CSerialPort::CSerialPort ALIAS cserialport_shared)
endif()

# ============================================================================
# 示例程序
# ============================================================================
if(CSERIALPORT_BUILD_EXAMPLES)
    # 创建示例目录
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/examples)
    
    # 示例1：基本使用
    add_executable(example_basic examples/example_basic.cpp)
    target_link_libraries(example_basic PRIVATE CSerialPort::CSerialPort)
    set_target_properties(example_basic PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/examples
    )
    
    # 示例2：枚举串口
    add_executable(example_enumerate examples/example_enumerate.cpp)
    target_link_libraries(example_enumerate PRIVATE CSerialPort::CSerialPort)
    set_target_properties(example_enumerate PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/examples
    )
    
    # 示例3：异步接收
    add_executable(example_async examples/example_async.cpp)
    target_link_libraries(example_async PRIVATE CSerialPort::CSerialPort)
    set_target_properties(example_async PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/examples
    )
endif()

# ============================================================================
# 安装
# ============================================================================
include(GNUInstallDirs)

# 安装头文件
install(FILES ${CSERIALPORT_HEADERS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cserialport
)

# 安装库
if(CSERIALPORT_BUILD_STATIC)
    install(TARGETS cserialport_static
        EXPORT CSerialPortTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
endif()

if(CSERIALPORT_BUILD_SHARED)
    install(TARGETS cserialport_shared
        EXPORT CSerialPortTargets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif()

# 安装CMake配置文件
install(EXPORT CSerialPortTargets
    FILE CSerialPortTargets.cmake
    NAMESPACE CSerialPort::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/CSerialPort
)

# 创建配置文件
include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/CSerialPortConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/CSerialPortConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/CSerialPort
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/CSerialPortConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/CSerialPortConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/CSerialPortConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/CSerialPort
)

# ============================================================================
# 打印配置摘要
# ============================================================================
message(STATUS "")
message(STATUS "CSerialPort Configuration Summary")
message(STATUS "==================================")
message(STATUS "  Version:        ${PROJECT_VERSION}")
message(STATUS "  Platform:       ${CSERIALPORT_PLATFORM}")
message(STATUS "  Build type:     ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard:   ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build shared:   ${CSERIALPORT_BUILD_SHARED}")
message(STATUS "  Build static:   ${CSERIALPORT_BUILD_STATIC}")
message(STATUS "  Build examples: ${CSERIALPORT_BUILD_EXAMPLES}")
message(STATUS "  Build tests:    ${CSERIALPORT_BUILD_TESTS}")
message(STATUS "")