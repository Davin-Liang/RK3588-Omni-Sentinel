cmake_minimum_required(VERSION 3.4.1)

project(dma_buffer_pool)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,--allow-shlib-undefined")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--allow-shlib-undefined") 
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置安装路径
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    message(STATUS "Running as STANDALONE project. Setting local install prefix.")
    set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install)
else()
    message(STATUS "Running as SUB-PROJECT. Using parent's install prefix: ${CMAKE_INSTALL_PREFIX}")
endif()


# RPATH 设置 (解决运行找不到库的问题)
set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/ 3rdparty.out)
if(NOT TARGET allocator_obj)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/allocator allocator.out)
endif()

# 编译库
add_library(dma_buffer_pool_lib STATIC
    src/dma-buffer-pool.cpp
)

target_include_directories(dma_buffer_pool_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(dma_buffer_pool_lib PUBLIC
    allocator_obj
)

# 编译可执行程序
# add_executable(usb_hikvisioner_demo
#     src/test_demo.cpp
# )

# target_link_libraries(usb_hikvisioner_demo
#     usb_hikvisioner_lib
# )
