set -e

TOOL_CHAIN=/home/elf/aarch64-buildroot-linux-gnu_sdk-buildroot
GCC_COMPILER=$TOOL_CHAIN/bin/aarch64-buildroot-linux-gnu

export LD_LIBRARY_PATH=${TOOL_CHAIN}/lib64:$LD_LIBRARY_PATH
export CC=${GCC_COMPILER}-gcc
export CXX=${GCC_COMPILER}-g++

# build
mkdir -p build/
cd build/
cmake ..
make -j4
make install
cd -
