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

# static library
add_library(lidar_camera_fusion_lib STATIC
    src/lidar_camera_fusion.cpp
)

target_include_directories(lidar_camera_fusion_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../sentinel-lslidarer/include
)

# demo executables
add_executable(lidar_camera_fusion_demo_single
    src/demo_single.cpp
)

add_executable(lidar_camera_fusion_demo_dual
    src/demo_dual.cpp
)

target_link_libraries(lidar_camera_fusion_demo_single lidar_camera_fusion_lib)
target_link_libraries(lidar_camera_fusion_demo_dual   lidar_camera_fusion_lib)

# install rules
install(TARGETS lidar_camera_fusion_lib           DESTINATION lib)
install(TARGETS lidar_camera_fusion_demo_single   DESTINATION ./)
install(TARGETS lidar_camera_fusion_demo_dual     DESTINATION ./)
install(DIRECTORY include/ DESTINATION include)
