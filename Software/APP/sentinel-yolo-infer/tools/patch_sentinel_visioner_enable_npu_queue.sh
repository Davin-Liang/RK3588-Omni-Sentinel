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
  echo "[OK] ${CPP} already pushes targetNpuBuf into npuTaskQueue. No patch needed."
  exit 0
fi

python3 - "${CPP}" <<'PY'
from pathlib import Path
import sys
p = Path(sys.argv[1])
s = p.read_text(encoding='utf-8')
old = '''                    if (npuOk) {
                        // TODO: NPU 推理接入后改为 npuTaskQueue.push(targetNpuBuf)
                        ctx->npuRgbPool->release_buffer(targetNpuBuf);
                    } else {
'''
new = '''                    if (npuOk) {
                        // 交给外部 NPU/YOLO 推理消费者；消费者处理完成后必须调用 release_npu(camNum, targetNpuBuf) 归还。
                        ctx->npuTaskQueue.push(targetNpuBuf);
                    } else {
'''
if old not in s:
    print('[ERROR] Expected code block not found; sentinel-visioner.cpp may have changed. Patch manually: replace release_buffer(targetNpuBuf) on npuOk path with npuTaskQueue.push(targetNpuBuf).')
    sys.exit(1)
backup = p.with_suffix(p.suffix + '.bak')
backup.write_text(s, encoding='utf-8')
p.write_text(s.replace(old, new), encoding='utf-8')
print(f'[OK] Patched {p}')
print(f'[OK] Backup saved to {backup}')
PY
