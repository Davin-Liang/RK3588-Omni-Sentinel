cmake_minimum_required(VERSION 3.4.1)

project(thermal_controller)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wl,--allow-shlib-undefined")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--allow-shlib-undefined")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    message(STATUS "Running as STANDALONE project. Setting local install prefix.")
    set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install)
else()
    message(STATUS "Running as SUB-PROJECT. Using parent's install prefix: ${CMAKE_INSTALL_PREFIX}")
endif()

set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

add_library(thermal_controller_lib STATIC
    src/thermal_controller.cpp
)

target_include_directories(thermal_controller_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
