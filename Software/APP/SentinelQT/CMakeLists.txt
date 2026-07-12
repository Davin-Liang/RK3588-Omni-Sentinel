cmake_minimum_required(VERSION 3.5)

project(SentinelQT LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install)
set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# Cross-compilation flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,--allow-shlib-undefined")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--allow-shlib-undefined")

# Qt5
if(DEFINED QT5_PREFIX)
    set(CMAKE_PREFIX_PATH ${QT5_PREFIX})
endif()
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# -----------------------------------------------------------------------------
# OpenCV include path for sub-projects
# -----------------------------------------------------------------------------
# SentinelQT 本身只保留 IMU-only EIS；但 sentinel-visioner 子工程的公共头文件
# 可能仍间接依赖 OpenCV include 路径。为保证 add_subdirectory 后子工程可编译，
# 此处保留全局 OpenCV include 路径。
set(APP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(PROJECT_OPENCV_INCLUDE_DIR ${APP_ROOT}/3rdparty/opencv/include)

if(EXISTS "${PROJECT_OPENCV_INCLUDE_DIR}/opencv2/opencv.hpp")
    include_directories(${PROJECT_OPENCV_INCLUDE_DIR})
    message(STATUS "Global OpenCV include dir for sub-projects: ${PROJECT_OPENCV_INCLUDE_DIR}")
else()
    message(WARNING "OpenCV header not found: ${PROJECT_OPENCV_INCLUDE_DIR}/opencv2/opencv.hpp")
endif()

# 第三方: RGA
if(NOT TARGET librga)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/librga 3rdparty_rga.out)
endif()

# 第三方: DMA Buffer Pool
if(NOT TARGET dma_buffer_pool_lib)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../dma-buffer-pool ${CMAKE_CURRENT_BINARY_DIR}/dma_build)
endif()

# 第三方: SentinelVisioner
if(NOT TARGET sentinel_visioner_lib)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../sentinel-visioner ${CMAKE_CURRENT_BINARY_DIR}/visioner_build)
endif()

# 第三方: SentinelStreamer (needs FFmpeg/libdrm)
set(FFMPEG_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/ffmpeg)
find_library(DRM_LIB drm)
if(NOT DRM_LIB)
    set(DRM_LIB drm)
endif()
if(NOT TARGET sentinel_streamer_lib)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../sentinel-streamer ${CMAKE_CURRENT_BINARY_DIR}/streamer_build)
endif()

# Lidar driver
if(NOT TARGET sentinel_lslidarer_lib)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../sentinel-lslidarer ${CMAKE_CURRENT_BINARY_DIR}/lslidarer_build)
endif()

# Lidar-Camera Fusion
if(NOT TARGET lidar_camera_fusion_lib)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../lidar-camera-fusion ${CMAKE_CURRENT_BINARY_DIR}/fusion_build)
endif()

# YOLO NPU 推理
if(NOT TARGET sentinel_yolo_infer_lib)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../sentinel-yolo-infer ${CMAKE_CURRENT_BINARY_DIR}/yolo_infer_build)
endif()

# Web 远程控制
if(NOT TARGET web_control_lib)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../web-control ${CMAKE_CURRENT_BINARY_DIR}/webcontrol_build)
endif()

# ICM45686 EIS 防抖
if(NOT TARGET icm45686_eis_lib)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../icm45686-eis-app ${CMAKE_CURRENT_BINARY_DIR}/icm45686_eis_build)
endif()

# NVMe SSD 回溯存储
if(NOT TARGET nvme_data_manager_lib)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../NVMe-SSD ${CMAKE_CURRENT_BINARY_DIR}/nvme_build)
endif()

# DeepSeek AI 推理
if(NOT TARGET deepseek_ai_lib)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../Deepseek ${CMAKE_CURRENT_BINARY_DIR}/deepseek_build)
endif()

find_package(Threads REQUIRED)

set(TS_FILES SentinelQT_zh_CN.ts)

add_executable(SentinelQT
    main.cpp
    widget.cpp
    widget.h
    widget.ui
    preview_worker.cpp
    preview_worker.h
    eis_quality_evaluator.cpp
    eis_quality_evaluator.h
    fusion_worker.cpp
    fusion_worker.h
    top_down_view.cpp
    top_down_view.h
    virtual_keyboard.cpp
    virtual_keyboard.h
    nvme_worker.cpp
    nvme_worker.h
    ${TS_FILES}
)

target_include_directories(SentinelQT PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../sentinel-visioner/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../sentinel-streamer/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../dma-buffer-pool/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../sentinel-lslidarer/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../lidar-camera-fusion/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../sentinel-yolo-infer/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../icm45686-eis-app/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../NVMe-SSD/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../Deepseek/include
    ${LIBRGA_INCLUDES}
)

target_link_libraries(SentinelQT PRIVATE
    Qt5::Widgets
    sentinel_streamer_lib
    sentinel_visioner_lib
    sentinel_lslidarer_lib
    lidar_camera_fusion_lib
    sentinel_yolo_infer_lib
    web_control_lib
    deepseek_ai_lib
    ${LIBRGA}
    Threads::Threads
    dma_buffer_pool_lib
    icm45686_eis_lib
    icm45686_user_lib
    nvme_data_manager_lib
)

if(Qt5LinguistTools_FOUND)
    qt5_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
endif()

install(TARGETS SentinelQT DESTINATION ./)
install(FILES config.ini ai_report.txt DESTINATION ./)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../web-control/web/ DESTINATION web
    FILES_MATCHING PATTERN "*.html" PATTERN "*.css" PATTERN "*.js")

