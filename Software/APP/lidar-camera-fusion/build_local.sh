#!/bin/bash
set -e

# 本地 x86 编译（不依赖交叉编译器，仅用于 Demo 测试）

mkdir -p build/
cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

echo ""
echo "Build done. Run:"
echo "  ./lidar_camera_fusion_demo_single"
echo "  ./lidar_camera_fusion_demo_dual"
