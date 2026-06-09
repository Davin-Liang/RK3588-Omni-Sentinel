cmake_minimum_required(VERSION 3.4.1)

project(sentinel_lslidarer)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,--allow-shlib-undefined")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--allow-shlib-undefined")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置安装路径
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    message(STATUS "Running as STANDALONE project. Setting local install prefix.")
    set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install)
else()
    message(STATUS "Running as SUB-PROJECT. Using parent's install prefix: ${CMAKE_INSTALL_PREFIX}")
endif()

# RPATH 设置
set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

find_package(Threads REQUIRED)

# 编译 SentinelLslidarer 静态库
add_library(sentinel_lslidarer_lib STATIC
    src/sentinel_lslidarer.cpp
    src/serial_port.cpp
    src/ring_buffer.cpp
    src/m10p_protocol.cpp
)

target_include_directories(sentinel_lslidarer_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(sentinel_lslidarer_lib PUBLIC
    Threads::Threads
)

# 编译 Demo 可执行程序
add_executable(sentinel_lslidarer_demo
    src/demo.cpp
)

target_link_libraries(sentinel_lslidarer_demo
    sentinel_lslidarer_lib
)

# 安装规则
install(TARGETS sentinel_lslidarer_lib DESTINATION lib)
install(TARGETS sentinel_lslidarer_demo DESTINATION ./)
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS lidar_udev.sh DESTINATION ./)
