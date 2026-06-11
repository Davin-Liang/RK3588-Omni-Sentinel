cmake_minimum_required(VERSION 3.10)

project(sentinel_yolo_infer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,--allow-shlib-undefined")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--allow-shlib-undefined")

option(SENTINEL_YOLO_BUILD_DEMO "Build sentinel-yolo-infer demo" ON)

find_package(Threads REQUIRED)

# ============================================================
# CMAKE_CURRENT_SOURCE_DIR = APP/sentinel-yolo-infer
# APP_ROOT_DIR             = APP
# ============================================================

set(APP_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." CACHE PATH "APP root directory")

set(SENTINEL_VISIONER_DIR
    "${APP_ROOT_DIR}/sentinel-visioner"
    CACHE PATH "Path to sentinel-visioner project"
)

set(RKNN_API_INCLUDE_DIR
    "${APP_ROOT_DIR}/3rdparty/rknpu2/include"
    CACHE PATH "Directory containing rknn_api.h"
)

set(RKNNRT_LIB
    "${APP_ROOT_DIR}/3rdparty/rknpu2/lib/librknnrt.so"
    CACHE FILEPATH "Path to librknnrt.so"
)

# ============================================================
# 检查 RKNN Runtime
# ============================================================

if(NOT EXISTS "${RKNN_API_INCLUDE_DIR}/rknn_api.h")
    message(FATAL_ERROR
        "Cannot find rknn_api.h at: ${RKNN_API_INCLUDE_DIR}/rknn_api.h"
    )
endif()

if(NOT EXISTS "${RKNNRT_LIB}")
    message(FATAL_ERROR
        "Cannot find librknnrt.so at: ${RKNNRT_LIB}"
    )
endif()

get_filename_component(RKNNRT_LIB_DIR "${RKNNRT_LIB}" DIRECTORY)

# 运行时优先从 APP/3rdparty/rknpu2/lib 找 librknnrt.so
set(CMAKE_BUILD_RPATH "${RKNNRT_LIB_DIR}")
set(CMAKE_INSTALL_RPATH "${RKNNRT_LIB_DIR}")

message(STATUS "APP_ROOT_DIR             = ${APP_ROOT_DIR}")
message(STATUS "SENTINEL_VISIONER_DIR    = ${SENTINEL_VISIONER_DIR}")
message(STATUS "RKNN_API_INCLUDE_DIR     = ${RKNN_API_INCLUDE_DIR}")
message(STATUS "RKNNRT_LIB               = ${RKNNRT_LIB}")

# ============================================================
# sentinel-visioner 依赖
# ============================================================

if(NOT TARGET sentinel_visioner_lib)
    if(EXISTS "${SENTINEL_VISIONER_DIR}/CMakeLists.txt")
        add_subdirectory(
            ${SENTINEL_VISIONER_DIR}
            ${CMAKE_CURRENT_BINARY_DIR}/sentinel_visioner.out
        )
    else()
        message(FATAL_ERROR
            "Cannot find sentinel-visioner at: ${SENTINEL_VISIONER_DIR}"
        )
    endif()
endif()

# ============================================================
# sentinel-yolo-infer 静态库
# ============================================================

add_library(sentinel_yolo_infer_lib STATIC
    src/SentinelYoloInfer.cpp
    src/Yolov8RknnEngine.cpp
)

target_include_directories(sentinel_yolo_infer_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${RKNN_API_INCLUDE_DIR}
)

target_include_directories(sentinel_yolo_infer_lib PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(sentinel_yolo_infer_lib PUBLIC
    sentinel_visioner_lib
    Threads::Threads
    dl
    ${RKNNRT_LIB}
)

# ============================================================
# demo
# ============================================================

if(SENTINEL_YOLO_BUILD_DEMO)
    add_executable(sentinel_yolo_infer_demo
        src/demo_yolo_infer.cpp
    )

    target_link_libraries(sentinel_yolo_infer_demo PRIVATE
        sentinel_yolo_infer_lib
    )
endif()

install(TARGETS sentinel_yolo_infer_lib DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(SENTINEL_YOLO_BUILD_DEMO)
    install(TARGETS sentinel_yolo_infer_demo DESTINATION bin)
endif()