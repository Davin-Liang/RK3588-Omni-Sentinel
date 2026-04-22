set -e

TOOL_CHAIN=/opt/tool_chain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu
GCC_COMPILER=$TOOL_CHAIN/bin/aarch64-linux-gnu

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
