#!/usr/bin/env bash
# 兼容旧入口；正式脚本位于 scripts/ur_dashboard_c161_recover.sh。

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RECOVERY_SCRIPT="${SCRIPT_DIR}/scripts/ur_dashboard_c161_recover.sh"

if [[ ! -x "$RECOVERY_SCRIPT" ]]; then
    printf '找不到正式恢复脚本: %s\n' "$RECOVERY_SCRIPT" >&2
    exit 1
fi

printf '提示: temp.sh 是兼容入口，后续请直接使用 %s\n' "$RECOVERY_SCRIPT"
exec "$RECOVERY_SCRIPT" --recover-c161
