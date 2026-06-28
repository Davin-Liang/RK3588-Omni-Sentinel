cmake_minimum_required(VERSION 3.4.1)

project(camera_calibrate)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,--allow-shlib-undefined")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--allow-shlib-undefined")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 安装路径: 本地 install/ 目录
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install)
endif()

# OpenCV (使用项目自带的 ARM64 预编译库)
set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/opencv/share/OpenCV)
find_package(OpenCV REQUIRED)

add_executable(camera_calibrate
    src/calibrate.cpp)

target_link_libraries(camera_calibrate
    ${OpenCV_LIBS})

install(TARGETS camera_calibrate DESTINATION ./)

# 连带拷贝 OpenCV .so 到 install/lib
set(OPENCV_LIB_SRC_DIR "${CMAKE_SOURCE_DIR}/../3rdparty/opencv/lib")
install(DIRECTORY ${OPENCV_LIB_SRC_DIR}/ DESTINATION lib
        FILES_MATCHING PATTERN "*.so*")
