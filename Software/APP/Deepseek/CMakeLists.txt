cmake_minimum_required(VERSION 3.5)

project(deepseek_ai LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Qt5 (AIReportWorker needs QObject/QThread)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# Threads (for std::mutex / std::condition_variable)
find_package(Threads REQUIRED)

set(SOURCES
    src/deepseek_inference.cpp
    src/ai_report_worker.cpp
)

set(HEADERS
    include/deepseek_inference.h
    include/ai_report_worker.h
)

add_library(deepseek_ai_lib STATIC ${SOURCES} ${HEADERS})

target_include_directories(deepseek_ai_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(deepseek_ai_lib PUBLIC
    Qt5::Widgets
    Threads::Threads
)

# ---- RKLLM Runtime (运行时 dlopen，不直接链接) ----
if(DEFINED RKLLM_RUNTIME_PATH)
    target_include_directories(deepseek_ai_lib PRIVATE
        ${RKLLM_RUNTIME_PATH}/include)
    # 不直接链接 librkllmrt.so，改为运行时 dlopen 加载
    # 仅需链接 dl 库提供 dlopen/dlsym
    target_link_libraries(deepseek_ai_lib PUBLIC ${CMAKE_DL_LIBS})
    target_compile_definitions(deepseek_ai_lib PUBLIC HAS_RKLLM)
    message(STATUS "DeepSeek AI: RKLLM Runtime via dlopen (${RKLLM_RUNTIME_PATH})")
else()
    message(STATUS "DeepSeek AI: RKLLM Runtime disabled (set -DRKLLM_RUNTIME_PATH to enable)")
endif()
