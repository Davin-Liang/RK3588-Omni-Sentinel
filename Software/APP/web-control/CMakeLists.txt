cmake_minimum_required(VERSION 3.5)

project(web_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Qt5 (need Widgets for QImage/QBuffer in MJPEG handler)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# Threads (for std::thread)
find_package(Threads REQUIRED)

set(SOURCES
    src/web_server.cpp
)

set(HEADERS
    include/web_server.h
    src/httplib.h
    src/json.hpp
)

add_library(web_control_lib STATIC ${SOURCES} ${HEADERS})

target_include_directories(web_control_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(web_control_lib PUBLIC
    Qt5::Widgets
    Threads::Threads
)
