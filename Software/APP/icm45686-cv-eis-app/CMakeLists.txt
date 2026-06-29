cmake_minimum_required(VERSION 3.4.1)

project(icm45686_eis_app C CXX)

# C 标准与 C++ 标准
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置安装路径：独立编译时安装到当前工程 install/ 目录
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    message(STATUS "Running as STANDALONE project. Setting local install prefix.")
    set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install)
else()
    message(STATUS "Running as SUB-PROJECT. Using parent's install prefix: ${CMAKE_INSTALL_PREFIX}")
endif()

# RPATH 设置：当前工程主要生成可执行文件，保留该配置便于后续扩展动态库
set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

find_package(Threads REQUIRED)

# 头文件路径
set(ICM45686_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

# 基础用户态访问库：封装 /dev/icm45686 open/read/ioctl
add_library(icm45686_user_lib STATIC
    src/icm45686_user.c
)

target_include_directories(icm45686_user_lib PUBLIC
    ${ICM45686_INCLUDE_DIR}
)

# EIS 应用层库：IMU读取线程、环形缓冲区、EIS像素偏移计算
add_library(icm45686_eis_lib STATIC
    src/imu_eis.cpp
)

target_include_directories(icm45686_eis_lib PUBLIC
    ${ICM45686_INCLUDE_DIR}
)

target_link_libraries(icm45686_eis_lib PUBLIC
    icm45686_user_lib
    Threads::Threads
    m
)

# 原始 IMU 数据与姿态角测试 Demo
add_executable(icm45686_app
    src/icm45686_app.c
    src/imu_ahrs.c
)

target_include_directories(icm45686_app PRIVATE
    ${ICM45686_INCLUDE_DIR}
)

target_link_libraries(icm45686_app PRIVATE
    icm45686_user_lib
    m
)

# EIS 防抖接口测试 Demo
add_executable(icm45686_eis_demo
    src/eis_demo.cpp
)

target_include_directories(icm45686_eis_demo PRIVATE
    ${ICM45686_INCLUDE_DIR}
)

target_link_libraries(icm45686_eis_demo PRIVATE
    icm45686_eis_lib
    icm45686_user_lib
    Threads::Threads
    m
)

# 离线画面抖动RMS评估工具
# 该工具依赖 OpenCV，用于比较 raw.mp4 和 eis.mp4 的帧间位移RMS抖动。
# 如果当前交叉编译环境中没有 OpenCV，则跳过该工具，不影响 IMU/EIS Demo 编译。
find_package(OpenCV QUIET)

if(OpenCV_FOUND)
    message(STATUS "OpenCV found: ${OpenCV_VERSION}. Building icm45686_jitter_eval.")

    add_library(vision_eis_lib STATIC
        src/vision_eis.cpp
    )

    target_include_directories(vision_eis_lib PUBLIC
        ${ICM45686_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
    )

    target_link_libraries(vision_eis_lib PUBLIC
        ${OpenCV_LIBS}
    )

    add_executable(icm45686_jitter_eval
        src/eis_jitter_eval.cpp
    )

    target_include_directories(icm45686_jitter_eval PRIVATE
        ${ICM45686_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
    )

    target_link_libraries(icm45686_jitter_eval PRIVATE
        ${OpenCV_LIBS}
    )

    add_executable(icm45686_vision_eis_offline
        src/vision_eis_offline_demo.cpp
    )

    target_include_directories(icm45686_vision_eis_offline PRIVATE
        ${ICM45686_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
    )

    target_link_libraries(icm45686_vision_eis_offline PRIVATE
        vision_eis_lib
        ${OpenCV_LIBS}
    )

    install(TARGETS icm45686_jitter_eval icm45686_vision_eis_offline DESTINATION ./)
else()
    message(WARNING "OpenCV not found. Skip icm45686_jitter_eval and icm45686_vision_eis_offline. "
                    "Install OpenCV or build these tools on PC if you need visual EIS/offline jitter evaluation.")
endif()

# 安装规则：安装到 install/ 目录根部，方便直接 scp 到板端
install(TARGETS icm45686_app DESTINATION ./)
install(TARGETS icm45686_eis_demo DESTINATION ./)
