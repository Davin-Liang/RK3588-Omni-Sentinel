add_library(allocator_obj OBJECT)

# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/ 3rdparty.out)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../librga 3rdparty_rga.out)
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../rknpu2 3rdparty_rknpu2.out)

target_sources(allocator_obj PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/dma_alloc.cpp
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dma_alloc.h
)

target_link_libraries(allocator_obj PUBLIC 
    # ${LIBRKNNRT}
    ${LIBRGA}
)

target_include_directories(allocator_obj PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../libdrm/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../libdrm/include/libdrm
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${LIBRGA_INCLUDES}
)
