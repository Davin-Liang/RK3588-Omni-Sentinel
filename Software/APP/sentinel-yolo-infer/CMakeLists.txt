cmake_minimum_required(VERSION 3.4.1)

project(sentinel_yolo_infer)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,--allow-shlib-undefined")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--allow-shlib-undefined")
# C++14 完全支持代码中使用的 std::thread、std::atomic、std::unique_ptr 等特性
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置安装路径
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    message(STATUS "Running as STANDALONE project. Setting local install prefix.")
    set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install)
else()
    message(STATUS "Running as SUB-PROJECT. Using parent's install prefix: ${CMAKE_INSTALL_PREFIX}")
endif()

# RPATH 设置，保证运行时可以找到 librknnrt.so
set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib;${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/rknpu2/lib")

find_package(Threads REQUIRED)

# RKNN Runtime
set(RKNN_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/rknpu2)
set(RKNN_API_INCLUDE_DIR ${RKNN_ROOT}/include)
set(RKNNRT_LIB ${RKNN_ROOT}/lib/librknnrt.so)

if(NOT EXISTS "${RKNN_API_INCLUDE_DIR}/rknn_api.h")
    message(FATAL_ERROR "Cannot find rknn_api.h: ${RKNN_API_INCLUDE_DIR}/rknn_api.h")
endif()

if(NOT EXISTS "${RKNNRT_LIB}")
    message(FATAL_ERROR "Cannot find librknnrt.so: ${RKNNRT_LIB}")
endif()

message(STATUS "RKNN_API_INCLUDE_DIR: ${RKNN_API_INCLUDE_DIR}")
message(STATUS "RKNNRT_LIB: ${RKNNRT_LIB}")

# 引入 sentinel-visioner
if(NOT TARGET sentinel_visioner_lib)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../sentinel-visioner ${CMAKE_CURRENT_BINARY_DIR}/sentinel_visioner.out)
endif()

# 编译 SentinelYoloInfer 静态库
add_library(sentinel_yolo_infer_lib STATIC
    src/SentinelYoloInfer.cpp
    src/Yolov8RknnEngine.cpp
)

# 设置头文件包含路径
target_include_directories(sentinel_yolo_infer_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${RKNN_API_INCLUDE_DIR}
)

target_include_directories(sentinel_yolo_infer_lib PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 链接对应的库
target_link_libraries(sentinel_yolo_infer_lib PUBLIC
    sentinel_visioner_lib
    Threads::Threads
    dl
    ${RKNNRT_LIB}
)

# 编译可执行测试程序 Demo
add_executable(sentinel_yolo_infer_demo
    src/demo_yolo_infer.cpp
)

# 链接库到可执行程序
target_link_libraries(sentinel_yolo_infer_demo
    sentinel_yolo_infer_lib
)

add_executable(sentinel_yolo_infer_dual_demo
    src/demo_yolo_infer_dual.cpp
)

target_link_libraries(sentinel_yolo_infer_dual_demo
    sentinel_yolo_infer_lib
)
# 安装规则
install(TARGETS sentinel_yolo_infer_lib DESTINATION lib)
install(TARGETS sentinel_yolo_infer_demo DESTINATION ./)
install(TARGETS sentinel_yolo_infer_dual_demo DESTINATION ./)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY models/ DESTINATION models)
