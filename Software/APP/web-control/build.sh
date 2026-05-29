#!/bin/bash
set -e

# web-control 静态库编译脚本
# 通常作为 SentinelQT 子目录编译，此脚本仅用于本地 x86 测试

BUILD_DIR="build"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

echo "web_control_lib built successfully"
