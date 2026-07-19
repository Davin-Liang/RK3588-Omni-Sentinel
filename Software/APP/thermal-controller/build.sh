#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
INSTALL_DIR="$SCRIPT_DIR/install"

# 交叉编译器路径 (按实际 SDK 路径修改)
TOOL_CHAIN=${TOOL_CHAIN:-/path/to/your/aarch64-buildroot-linux-gnu_sdk-buildroot}

export CC="${TOOL_CHAIN}/bin/aarch64-buildroot-linux-gnu-gcc"
export CXX="${TOOL_CHAIN}/bin/aarch64-buildroot-linux-gnu-g++"

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR"

make -j$(nproc)
make install

echo "Build complete: $INSTALL_DIR"
