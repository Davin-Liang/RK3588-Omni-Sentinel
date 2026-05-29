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

find_package(Threads REQUIRED)

set(TS_FILES SentinelQT_zh_CN.ts)

add_executable(SentinelQT
    main.cpp
    widget.cpp
    widget.h
    widget.ui
    preview_worker.cpp
    preview_worker.h
    fusion_worker.cpp
    fusion_worker.h
    top_down_view.cpp
    top_down_view.h
    virtual_keyboard.cpp
    virtual_keyboard.h
    ${TS_FILES}
)

target_include_directories(SentinelQT PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../sentinel-visioner/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../sentinel-streamer/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../dma-buffer-pool/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../sentinel-lslidarer/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../lidar-camera-fusion/include
    ${LIBRGA_INCLUDES}
)

target_link_libraries(SentinelQT PRIVATE
    Qt5::Widgets
    sentinel_streamer_lib
    sentinel_visioner_lib
    sentinel_lslidarer_lib
    lidar_camera_fusion_lib
    ${LIBRGA}
    Threads::Threads
    dma_buffer_pool_lib
)

if(Qt5LinguistTools_FOUND)
    qt5_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
endif()

install(TARGETS SentinelQT DESTINATION ./)
install(FILES config.ini DESTINATION ./)
