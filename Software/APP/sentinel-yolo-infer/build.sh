#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
APP_ROOT_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)

BUILD_DIR="${SCRIPT_DIR}/build"
INSTALL_DIR="${SCRIPT_DIR}/install"

mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
  -DAPP_ROOT_DIR="${APP_ROOT_DIR}" \
  -DSENTINEL_VISIONER_DIR="${APP_ROOT_DIR}/sentinel-visioner" \
  -DRKNN_API_INCLUDE_DIR="${APP_ROOT_DIR}/3rdparty/rknpu2/include" \
  -DRKNNRT_LIB="${APP_ROOT_DIR}/3rdparty/rknpu2/lib/librknnrt.so" \
  "$@"

cmake --build . -j"$(nproc)"
cmake --install .