cmake_minimum_required(VERSION 3.15)

project(3rdparty_rga_libs)

# rga
set(RGA_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(LIBRGA ${RGA_PATH}/lib/librga.a PARENT_SCOPE)
set(LIBRGA_INCLUDES ${RGA_PATH}/include PARENT_SCOPE)
install(PROGRAMS ${RGA_PATH}/lib/librga.so DESTINATION lib)
