cmake_minimum_required(VERSION 3.4.1)

project(sentinel_visioner)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,--allow-shlib-undefined")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--allow-shlib-undefined") 
# C++14 完全支持代码中使用的 std::make_unique 和 std::thread 等特性
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置安装路径
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    message(STATUS "Running as STANDALONE project. Setting local install prefix.")
    set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install)
else()
    message(STATUS "Running as SUB-PROJECT. Using parent's install prefix: ${CMAKE_INSTALL_PREFIX}")
endif()

# RPATH 设置 (解决运行找不到共享库的问题)
set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# OpenCV 配置 (通常用于 Demo 中的 cv::imshow 或者图像保存验证)
set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/opencv/share/OpenCV)
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV libraries found: ${OpenCV_LIBS}")
set(OPENCV_LIB_SRC_DIR "${CMAKE_SOURCE_DIR}/../3rdparty/opencv/lib")

# 引入第三方库支持
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/librga 3rdparty_rga.out)

# 引入你的 DMA Buffer Pool 库
if(NOT TARGET dma_buffer_pool_lib)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../dma-buffer-pool ${CMAKE_CURRENT_BINARY_DIR}/dma_build)
endif()

# 编译 SentinelVisioner 静态库
add_library(sentinel_visioner_lib STATIC
    src/sentinel-visioner.cpp
)

# 设置头文件包含路径
target_include_directories(sentinel_visioner_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${LIBRGA_INCLUDES}
)

find_package(Threads REQUIRED)

# 链接对应的库
target_link_libraries(sentinel_visioner_lib PUBLIC
    ${LIBRGA}
    Threads::Threads
    dma_buffer_pool_lib
)

# 编译可执行测试程序 (Demo)
add_executable(sentinel_visioner_demo1
    src/demo1.cpp
)

# 链接库到可执行程序
target_link_libraries(sentinel_visioner_demo1
    sentinel_visioner_lib
    # ${OpenCV_LIBS}
)

# 安装规则
install(TARGETS sentinel_visioner_demo1 DESTINATION ./)
# install(DIRECTORY ${OPENCV_LIB_SRC_DIR}/ DESTINATION lib)