cmake_minimum_required(VERSION 3.4.1)

project(sentinel_streamer)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,--allow-shlib-undefined")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--allow-shlib-undefined")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- install prefix: standalone vs sub-project ----
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    message(STATUS "Running as STANDALONE project. Setting local install prefix.")
    set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install)
else()
    message(STATUS "Running as SUB-PROJECT. Using parent's install prefix: ${CMAKE_INSTALL_PREFIX}")
endif()

# ---- RPATH ----
set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib;${FFMPEG_ROOT}/lib")

# ---- 第三方依赖: RGA ----
if(NOT TARGET librga)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/librga 3rdparty_rga.out)
endif()

# ---- 第三方依赖: DMA Buffer Pool ----
if(NOT TARGET dma_buffer_pool_lib)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../dma-buffer-pool ${CMAKE_CURRENT_BINARY_DIR}/dma_build)
endif()

# ---- 第三方依赖: SentinelVisioner ----
if(NOT TARGET sentinel_visioner_lib)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../sentinel-visioner ${CMAKE_CURRENT_BINARY_DIR}/visioner_build)
endif()

# ---- 第三方依赖: FFmpeg (交叉编译产物，位于 3rdparty/ffmpeg) ----
set(FFMPEG_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/ffmpeg)

# ---- Threads ----
find_package(Threads REQUIRED)

# ---- OpenCV（sentinel-visioner 头文件的传递依赖） ----
set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/opencv/share/OpenCV)
find_package(OpenCV REQUIRED)

set(AVCODEC_LIB  ${FFMPEG_ROOT}/lib/libavcodec.so)
set(AVFORMAT_LIB ${FFMPEG_ROOT}/lib/libavformat.so)
set(AVUTIL_LIB   ${FFMPEG_ROOT}/lib/libavutil.so)

# ---- libdrm (DRM_FORMAT_NV12 / DRM_FORMAT_MOD_LINEAR，交叉编译器 sysroot 自带) ----
find_library(DRM_LIB drm)
if(NOT DRM_LIB)
    message(WARNING "libdrm not found, will attempt direct linking.")
    set(DRM_LIB drm)
endif()

# ---- 编译 SentinelStreamer 静态库 ----
add_library(sentinel_streamer_lib STATIC
    src/sentinel_streamer.cpp
    src/rga_scaler.cpp
    src/mpp_encoder.cpp
    src/record_buffer_pool.cpp
)
# ---- 第三方依赖: ICM45686 EIS ----
if(NOT TARGET icm45686_eis_lib)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../icm45686-eis-app
                     ${CMAKE_CURRENT_BINARY_DIR}/icm45686_eis_build)
endif()

target_include_directories(sentinel_streamer_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../sentinel-visioner/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../dma-buffer-pool/include
    ${FFMPEG_ROOT}/include
    ${LIBRGA_INCLUDES}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(sentinel_streamer_lib PUBLIC
    ${LIBRGA}
    ${AVCODEC_LIB}
    ${AVFORMAT_LIB}
    ${AVUTIL_LIB}
    ${DRM_LIB}
    Threads::Threads
    dma_buffer_pool_lib
)

# ---- Demo 可执行文件 ----
add_executable(sentinel_streamer_demo
    src/demo_stream.cpp
)

add_executable(sentinel_streamer_demo_cycle
    src/demo_cycle.cpp
)

target_link_libraries(sentinel_streamer_demo
    sentinel_streamer_lib
    sentinel_visioner_lib
)

target_link_libraries(sentinel_streamer_demo_cycle
    sentinel_streamer_lib
    sentinel_visioner_lib
)

set_target_properties(sentinel_streamer_demo_cycle PROPERTIES
    LINK_FLAGS "-rdynamic")

# ---- 安装规则 ----
add_executable(sentinel_record_pool_test
    src/demo_record_pool.cpp
)

target_include_directories(sentinel_record_pool_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../dma-buffer-pool/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../sentinel-visioner/include
)

target_link_libraries(sentinel_record_pool_test
    sentinel_streamer_lib
    sentinel_visioner_lib
    dma_buffer_pool_lib
)

install(TARGETS sentinel_streamer_lib  DESTINATION lib)
install(TARGETS sentinel_streamer_demo DESTINATION ./)
install(TARGETS sentinel_streamer_demo_cycle DESTINATION ./)
install(TARGETS sentinel_record_pool_test DESTINATION ./)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY ${FFMPEG_ROOT}/lib/ DESTINATION lib
    FILES_MATCHING PATTERN "*.so*")
