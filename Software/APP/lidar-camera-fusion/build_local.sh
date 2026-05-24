#!/bin/bash
set -e

# x86 local build (no cross-compiler, demo test only)

mkdir -p build/
cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc 2>/dev/null || echo 4)

echo ""
echo "Build done. Run:"
echo "  ./lidar_camera_fusion_demo_single"
echo "  ./lidar_camera_fusion_demo_dual"
echo "  ./lidar_camera_fusion_demo_tracking"
