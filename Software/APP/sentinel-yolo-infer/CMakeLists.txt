cmake_minimum_required(VERSION 3.10)

project(sentinel_yolo_infer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--allow-shlib-undefined")

option(SENTINEL_YOLO_BUILD_DEMO "Build demo executable" ON)

# 默认工程结构：
# workspace/
#   sentinel-visioner/
#   sentinel-yolo-infer/
# 可通过 -DSENTINEL_VISIONER_DIR=/path/to/sentinel-visioner 覆盖。
set(WORKSPACE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." CACHE PATH "Workspace root directory")
set(SENTINEL_VISIONER_DIR "${WORKSPACE_DIR}/sentinel-visioner" CACHE PATH "Path to sentinel-visioner project")

# RKNN Runtime 并不要求目录名一定叫 rknpu2；只要能找到 rknn_api.h 和 librknnrt.so 即可。
# 如果板端已经安装到系统路径，通常无需传参；否则可使用：
#   -DRKNN_ROOT=/path/to/rknpu2
# 或分别指定：
#   -DRKNN_API_INCLUDE_DIR=/path/to/include -DRKNNRT_LIB=/path/to/librknnrt.so
set(RKNN_ROOT "" CACHE PATH "Optional RKNN/rknpu2 root directory")
set(RKNN_API_INCLUDE_DIR "" CACHE PATH "Directory containing rknn_api.h")
set(RKNNRT_LIB "" CACHE FILEPATH "Path to librknnrt.so")

find_package(Threads REQUIRED)

# ---------- RKNN auto-detect ----------
if(NOT RKNN_API_INCLUDE_DIR)
    set(_rknn_include_candidates
        "${RKNN_ROOT}/runtime/Linux/librknn_api/include"
        "${WORKSPACE_DIR}/rknpu2/runtime/Linux/librknn_api/include"
        "/usr/include"
        "/usr/local/include"
        "/usr/include/rknn"
        "/usr/local/include/rknn"
    )
    foreach(_dir ${_rknn_include_candidates})
        if(EXISTS "${_dir}/rknn_api.h")
            set(RKNN_API_INCLUDE_DIR "${_dir}" CACHE PATH "Directory containing rknn_api.h" FORCE)
            break()
        endif()
    endforeach()
endif()

if(NOT RKNNRT_LIB)
    set(_rknn_lib_candidates
        "${RKNN_ROOT}/runtime/Linux/librknn_api/aarch64/librknnrt.so"
        "${WORKSPACE_DIR}/rknpu2/runtime/Linux/librknn_api/aarch64/librknnrt.so"
        "/usr/lib/librknnrt.so"
        "/usr/local/lib/librknnrt.so"
        "/usr/lib/aarch64-linux-gnu/librknnrt.so"
        "/usr/local/lib/aarch64-linux-gnu/librknnrt.so"
    )
    foreach(_lib ${_rknn_lib_candidates})
        if(EXISTS "${_lib}")
            set(RKNNRT_LIB "${_lib}" CACHE FILEPATH "Path to librknnrt.so" FORCE)
            break()
        endif()
    endforeach()
endif()

if(NOT EXISTS "${RKNN_API_INCLUDE_DIR}/rknn_api.h")
    message(FATAL_ERROR
        "Cannot find rknn_api.h. Install RKNN Runtime or pass -DRKNN_ROOT=/path/to/rknpu2 "
        "or -DRKNN_API_INCLUDE_DIR=/path/to/librknn_api/include")
endif()

# ---------- sentinel-visioner dependency ----------
# 如果上层工程已经 add_subdirectory(sentinel-visioner) 并定义 sentinel_visioner_lib，
# 这里不会重复添加；否则默认从兄弟目录自动添加。
if(NOT TARGET sentinel_visioner_lib)
    if(EXISTS "${SENTINEL_VISIONER_DIR}/CMakeLists.txt")
        add_subdirectory(${SENTINEL_VISIONER_DIR} ${CMAKE_CURRENT_BINARY_DIR}/sentinel_visioner.out)
    else()
        message(FATAL_ERROR
            "Cannot find sentinel-visioner. Expected at ${SENTINEL_VISIONER_DIR}. "
            "Keep workspace/sentinel-visioner and workspace/sentinel-yolo-infer as sibling directories, "
            "or pass -DSENTINEL_VISIONER_DIR=/path/to/sentinel-visioner")
    endif()
endif()

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
)

if(RKNNRT_LIB)
    target_link_libraries(sentinel_yolo_infer_lib PUBLIC ${RKNNRT_LIB})
else()
    # 若系统默认链接路径中已有 librknnrt.so，可直接链接 rknnrt。
    target_link_libraries(sentinel_yolo_infer_lib PUBLIC rknnrt)
endif()

if(SENTINEL_YOLO_BUILD_DEMO)
    add_executable(sentinel_yolo_infer_demo
        src/demo_yolo_infer.cpp
    )
    target_link_libraries(sentinel_yolo_infer_demo PRIVATE sentinel_yolo_infer_lib)
endif()

install(TARGETS sentinel_yolo_infer_lib DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
if(SENTINEL_YOLO_BUILD_DEMO)
    install(TARGETS sentinel_yolo_infer_demo DESTINATION bin)
endif()
