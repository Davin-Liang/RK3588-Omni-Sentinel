set -e

TOOL_CHAIN=/home/topeet/Linux_SDK/linux_sdk/prebuilts/gcc/linux-x86/aarch64/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu
GCC_COMPILER=$TOOL_CHAIN/bin/aarch64-linux-gnu

export LD_LIBRARY_PATH=${TOOL_CHAIN}/lib64:$LD_LIBRARY_PATH
export CC=${GCC_COMPILER}-gcc
export CXX=${GCC_COMPILER}-g++

# 构建
mkdir -p build/
cd build/
cmake ..
make -j4
make install
cd -