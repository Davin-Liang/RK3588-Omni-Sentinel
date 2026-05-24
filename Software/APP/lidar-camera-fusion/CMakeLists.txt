cmake_minimum_required(VERSION 3.4.1)

project(lidar_camera_fusion)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,--allow-shlib-undefined")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--allow-shlib-undefined")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# install prefix: standalone vs sub-project
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    message(STATUS "Running as STANDALONE project. Setting local install prefix.")
    set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install)
else()
    message(STATUS "Running as SUB-PROJECT. Using parent's install prefix: ${CMAKE_INSTALL_PREFIX}")
endif()

# RPATH
set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

find_package(Threads REQUIRED)

# === 核心库（纯算法，无线程，demo 用） ===
add_library(lidar_camera_fusion_core STATIC
    src/lidar_camera_fusion.cpp
    src/lidar_target_tracker.cpp
)

target_include_directories(lidar_camera_fusion_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../sentinel-lslidarer/include
)

# === 完整库（核心 + 线程，实际部署用，依赖 sentinel-lslidarer） ===
add_library(lidar_camera_fusion_lib STATIC
    src/lidar_camera_fusion.cpp
    src/lidar_camera_fusion_thread.cpp
    src/lidar_target_tracker.cpp
)

target_include_directories(lidar_camera_fusion_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../sentinel-lslidarer/include
)

target_link_libraries(lidar_camera_fusion_lib PUBLIC Threads::Threads)

# === demo executables（只依赖核心库，可在 x86 本地编译测试） ===
add_executable(lidar_camera_fusion_demo_single
    src/demo_single.cpp
)

add_executable(lidar_camera_fusion_demo_dual
    src/demo_dual.cpp
)

target_link_libraries(lidar_camera_fusion_demo_single lidar_camera_fusion_core)
target_link_libraries(lidar_camera_fusion_demo_dual   lidar_camera_fusion_core)

# === demo_tracking（跟踪测试，x86 本地可编译） ===
add_executable(lidar_camera_fusion_demo_tracking
    src/demo_tracking.cpp
)

target_link_libraries(lidar_camera_fusion_demo_tracking lidar_camera_fusion_core)

# === demo_thread（目标板运行，依赖真实雷达驱动） ===
if(NOT TARGET sentinel_lslidarer_lib)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../sentinel-lslidarer
                     ${CMAKE_CURRENT_BINARY_DIR}/lslidarer_build)
endif()

add_executable(lidar_camera_fusion_demo_thread
    src/demo_thread.cpp
)

target_link_libraries(lidar_camera_fusion_demo_thread
    lidar_camera_fusion_lib
    sentinel_lslidarer_lib
    Threads::Threads
)

# === install rules ===
install(TARGETS lidar_camera_fusion_lib           DESTINATION lib)
install(TARGETS lidar_camera_fusion_demo_single   DESTINATION ./)
install(TARGETS lidar_camera_fusion_demo_dual     DESTINATION ./)
install(TARGETS lidar_camera_fusion_demo_thread   DESTINATION ./)
install(DIRECTORY include/ DESTINATION include)
