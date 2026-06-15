#!/usr/bin/env bash
set -euo pipefail

VISIONER_DIR="${1:-../sentinel-visioner}"
CPP="${VISIONER_DIR}/src/sentinel-visioner.cpp"

if [[ ! -f "${CPP}" ]]; then
  echo "[ERROR] Cannot find ${CPP}"
  echo "Usage: $0 /path/to/sentinel-visioner"
  exit 2
fi

if grep -q "npuTaskQueue.push(targetNpuBuf)" "${CPP}"; then
  echo "[OK] sentinel-visioner pushes NPU RGB buffers into npuTaskQueue."
  exit 0
fi

if grep -q "改为 npuTaskQueue.push(targetNpuBuf)" "${CPP}" && grep -q "npuRgbPool->release_buffer(targetNpuBuf)" "${CPP}"; then
  echo "[WARN] Current sentinel-visioner converts NPU RGB buffers but releases them immediately."
  echo "       SentinelYoloInfer will wait on try_get_npu()/wait_get_npu() and receive no frames."
  echo "       Run: tools/patch_sentinel_visioner_enable_npu_queue.sh ${VISIONER_DIR}"
  exit 1
fi

echo "[WARN] Cannot determine NPU queue behavior automatically. Please inspect ${CPP}."
exit 1
