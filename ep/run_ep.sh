#!/usr/bin/env bash
# EP 真机一键跑: bash ep/run_ep.sh [config] [cycles]
#   例: bash ep/run_ep.sh ep/config_ep.json 1    # 首跑 1 周期盯全程
#        bash ep/run_ep.sh ep/config_ep.json 5    # C5 验收(>=4/5)
set -euo pipefail
cd "$(dirname "$0")/.."
CONFIG="${1:-ep/config_ep.json}"
CYCLES="${2:-}"

echo "[1/2] 离线自检: python3 ep/core/selfcheck.py --config $CONFIG"
python3 ep/core/selfcheck.py --config "$CONFIG"

ARGS=(--config "$CONFIG")
if [ -n "$CYCLES" ]; then
    ARGS+=(--cycles "$CYCLES")
fi
echo "[2/2] python3 ep/drive/ep_cycle.py ${ARGS[*]}"
exec python3 ep/drive/ep_cycle.py "${ARGS[@]}"
