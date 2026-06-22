cmake_minimum_required(VERSION 3.4.1)

project(nvme_ssd_test)

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

# ---- FFmpeg（MPP 硬件编码 + MP4 复用，替代 OpenCV） ----
set(FFMPEG_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/ffmpeg)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib;${FFMPEG_ROOT}/lib")

# 编译 NVMeDataManager 静态库
add_library(nvme_data_manager_lib STATIC
    src/NVMeDataManager.cpp
)

# 设置头文件包含路径
target_include_directories(nvme_data_manager_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${FFMPEG_ROOT}/include
)

link_directories(${FFMPEG_ROOT}/lib)

# 查找线程库
find_package(Threads REQUIRED)

# 链接对应的库
target_link_libraries(nvme_data_manager_lib PUBLIC
    Threads::Threads
    ${FFMPEG_ROOT}/lib/libavcodec.so
    ${FFMPEG_ROOT}/lib/libavformat.so
    ${FFMPEG_ROOT}/lib/libavutil.so
    ${FFMPEG_ROOT}/lib/libswscale.so
)

# 编译可执行测试程序 (Demo)
add_executable(nvme_demo EXCLUDE_FROM_ALL
    src/nvme_demo.cpp
)

# 编译可执行 Benchmark 程序
add_executable(nvme_benchmark EXCLUDE_FROM_ALL
    src/nvme_benchmark.cpp
)

# 链接库到可执行程序
target_link_libraries(nvme_demo
    nvme_data_manager_lib
)
target_link_libraries(nvme_benchmark
    nvme_data_manager_lib
)

# 安装规则 (demo/benchmark 仅 standalone 构建时安装)
install(TARGETS nvme_data_manager_lib DESTINATION lib)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    install(TARGETS nvme_demo nvme_benchmark DESTINATION ./)
endif()
install(DIRECTORY ${FFMPEG_ROOT}/lib/ DESTINATION lib
    FILES_MATCHING PATTERN "*.so*")

# 输出信息
message(STATUS "NVMe SSD Demo project configured successfully")
message(STATUS "Build command: cd build && cmake .. && make -j4")
message(STATUS "Run demo: ./nvme_demo (requires root privileges for NVMe access)")