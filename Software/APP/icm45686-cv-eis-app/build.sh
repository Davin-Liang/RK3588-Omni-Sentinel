#!/bin/bash
set -e

# RK3588 Buildroot 交叉编译工具链路径，可按实际环境修改
TOOL_CHAIN=${TOOL_CHAIN:-/home/elf/aarch64-buildroot-linux-gnu_sdk-buildroot}
GCC_COMPILER=${GCC_COMPILER:-$TOOL_CHAIN/bin/aarch64-buildroot-linux-gnu}

export LD_LIBRARY_PATH=${TOOL_CHAIN}/lib64:$LD_LIBRARY_PATH
export CC=${CC:-${GCC_COMPILER}-gcc}
export CXX=${CXX:-${GCC_COMPILER}-g++}

# build
mkdir -p build/
cd build/
cmake ..
make -j4
make install
cd -

echo ""
echo "Build finished. Output files are in: install/"
echo "  install/icm45686_app"
echo "  install/icm45686_eis_demo"
