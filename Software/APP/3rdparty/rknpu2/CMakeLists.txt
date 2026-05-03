cmake_minimum_required(VERSION 3.15)

project(3rdparty_rknpu2_libs)

# for rknpu2
set(RKNN_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(LIBRKNNRT ${RKNN_PATH}/lib/librknnrt.so PARENT_SCOPE)
set(LIBRKNNRT_INCLUDES ${RKNN_PATH}/include PARENT_SCOPE)
install(PROGRAMS ${RKNN_PATH}/lib/librknnrt.so DESTINATION lib)
