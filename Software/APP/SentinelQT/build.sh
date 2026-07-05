#!/bin/bash
set -e

TOOL_CHAIN=${CROSS_COMPILE_PATH:-/home/elf/aarch64-buildroot-linux-gnu_sdk-buildroot}
GCC_COMPILER=$TOOL_CHAIN/bin/aarch64-buildroot-linux-gnu

export LD_LIBRARY_PATH=${TOOL_CHAIN}/lib64:$LD_LIBRARY_PATH
export CC=${GCC_COMPILER}-gcc
export CXX=${GCC_COMPILER}-g++

# Qt5 cmake module path in SDK
QT5_CMAKE_DIR=${QT5_PREFIX:-"$TOOL_CHAIN/aarch64-buildroot-linux-gnu/sysroot/usr/lib/cmake/Qt5"}

mkdir -p build/
cd build/
# DeepSeek / RKLLM Runtime SDK path (optional)
# 指向 rkllm-runtime SDK 的 librkllm_api 目录
# 例如: /home/elf/Linux_SDK/Deepseek/Deepseek_env/rknn-llm-main/rkllm-runtime/Linux/librkllm_api
RKLLM_RUNTIME_PATH=${RKLLM_RUNTIME_PATH:-""}

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_SYSROOT="$TOOL_CHAIN/aarch64-buildroot-linux-gnu/sysroot" \
    -DCMAKE_PREFIX_PATH="$QT5_CMAKE_DIR" \
    -DRKLLM_RUNTIME_PATH="$RKLLM_RUNTIME_PATH"
make -j$(nproc)
make install
