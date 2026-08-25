#!/usr/bin/env bash
# UR3/CB3 C161 人工恢复工具。
# 默认只读诊断；只有 --recover-c161 且人工输入确认词后才会解除保护停机。

set -uo pipefail

ROBOT_IP="${UR_ROBOT_IP:-192.168.131.3}"
DASHBOARD_PORT="${UR_DASHBOARD_PORT:-29999}"
QUERY_TIMEOUT="${UR_DASHBOARD_QUERY_TIMEOUT:-8}"
STATE_TIMEOUT="${UR_DASHBOARD_STATE_TIMEOUT:-15}"
POLL_INTERVAL="${UR_DASHBOARD_POLL_INTERVAL:-0.5}"
LOG_FILE="${UR_DASHBOARD_RECOVERY_LOG:-${HOME}/.ros/ur_dashboard_c161_recovery.log}"

mkdir -p "$(dirname "$LOG_FILE")"

log() {
    local level="$1"
    shift
    printf '[%s] [%s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$level" "$*" | tee -a "$LOG_FILE"
}

usage() {
    cat <<'EOF'
用法:
  ./ur_dashboard_c161_recover.sh --diagnose      只读查询控制器状态（默认）
  ./ur_dashboard_c161_recover.sh --recover-c161 人工确认后解除 C161 保护停机
  ./ur_dashboard_c161_recover.sh --help          显示帮助

安全约束:
  - 本脚本不能从 Dashboard 自动识别具体错误码；--recover-c161 仅用于现场已确认是 C161。
  - 解锁前必须确认编码器姿态与实机一致、工作区无人、急停可用。
  - 本脚本不会发送 power on、brake release、load 或 play。

可选环境变量:
  UR_ROBOT_IP                 默认 192.168.131.3
  UR_DASHBOARD_PORT           默认 29999
  UR_DASHBOARD_RECOVERY_LOG   默认 ~/.ros/ur_dashboard_c161_recovery.log
EOF
}

dashboard_query() {
    local command="$1"
    timeout "$QUERY_TIMEOUT" python3 - "$ROBOT_IP" "$DASHBOARD_PORT" "$command" <<'PY'
import socket
import sys

host, port, command = sys.argv[1], int(sys.argv[2]), sys.argv[3]
with socket.create_connection((host, port), timeout=3.0) as sock:
    sock.settimeout(3.0)
    sock.recv(4096)
    sock.sendall((command + "\n").encode("ascii"))
    print(sock.recv(4096).decode("utf-8", errors="replace").strip())
PY
}

query_required() {
    local command="$1"
    local reply
    if ! reply="$(dashboard_query "$command" 2>&1)"; then
        log ERROR "Dashboard 命令失败: ${command}; ${reply}"
        return 1
    fi
    printf '%s\n' "$reply"
}

snapshot() {
    local label="$1"
    local version_reply robot_reply safety_reply program_reply

    version_reply="$(query_required PolyscopeVersion)" || return 1
    robot_reply="$(query_required robotmode)" || return 1
    safety_reply="$(query_required safetymode)" || return 1
    program_reply="$(query_required programState)" || return 1

    CURRENT_ROBOT_MODE="${robot_reply#Robotmode: }"
    CURRENT_SAFETY_MODE="${safety_reply#Safetymode: }"
    CURRENT_PROGRAM_STATE="${program_reply%% *}"

    log INFO "${label}: ${version_reply}"
    log STATE "${label}: robot=${CURRENT_ROBOT_MODE}, safety=${CURRENT_SAFETY_MODE}, program=${program_reply}"
}

wait_safety_normal() {
    local deadline=$((SECONDS + STATE_TIMEOUT))
    local reply safety last=""

    while (( SECONDS < deadline )); do
        reply="$(query_required safetymode)" || return 1
        safety="${reply#Safetymode: }"
        if [[ "$safety" != "$last" ]]; then
            log STATE "解锁后安全状态: ${safety}"
            last="$safety"
        fi
        if [[ "$safety" == "NORMAL" || "$safety" == "REDUCED" ]]; then
            return 0
        fi
        sleep "$POLL_INTERVAL"
    done

    log ERROR "解锁请求已发送，但 ${STATE_TIMEOUT}s 内安全状态未恢复为 NORMAL/REDUCED。"
    return 1
}

recover_c161() {
    local confirmation reply

    snapshot "解锁前" || return 1

    if [[ "$CURRENT_SAFETY_MODE" == "NORMAL" || "$CURRENT_SAFETY_MODE" == "REDUCED" ]]; then
        log INFO "安全状态已经正常，无需解锁。"
        return 0
    fi
    if [[ "$CURRENT_SAFETY_MODE" != "PROTECTIVE_STOP" ]]; then
        log ERROR "当前状态为 ${CURRENT_SAFETY_MODE}，不是 PROTECTIVE_STOP；拒绝发送解锁命令。"
        log ERROR "FAULT/VIOLATION/急停等状态必须按对应故障处理。"
        return 1
    fi
    if [[ ! -t 0 ]]; then
        log ERROR "恢复操作必须在交互式终端中人工确认，不能由 roslaunch 或后台任务自动执行。"
        return 1
    fi

    printf '\n仅当屏幕/日志确认为 C161，且编码器姿态与实机一致时才能继续。\n'
    printf '确认工作区无人、急停可用后，输入 C161-VERIFIED：'
    IFS= read -r confirmation
    if [[ "$confirmation" != "C161-VERIFIED" ]]; then
        log WARN "确认词不匹配，未发送任何解锁命令。"
        return 2
    fi

    reply="$(query_required 'unlock protective stop')" || return 1
    log ACTION "unlock protective stop -> ${reply}"

    if [[ "$reply" == *"until 5s"* ]]; then
        log WARN "控制器要求等待 5 秒，将等待后重试一次。"
        sleep 5
        reply="$(query_required 'unlock protective stop')" || return 1
        log ACTION "unlock protective stop (retry) -> ${reply}"
    fi
    if [[ "$reply" != *"Protective stop releasing"* ]]; then
        log ERROR "控制器未接受解锁请求，不执行后续启动操作。"
        return 1
    fi

    wait_safety_normal || return 1
    snapshot "解锁后" || return 1
    log INFO "C161 保护停机已解除。本工具没有上电、松闸或启动程序。"
    log INFO "下一步运行: $(dirname "${BASH_SOURCE[0]}")/ur_dashboard_startup.sh"
}

main() {
    case "${1:---diagnose}" in
        --diagnose)
            log INFO "执行只读 Dashboard 诊断: ${ROBOT_IP}:${DASHBOARD_PORT}"
            snapshot "当前状态"
            ;;
        --recover-c161)
            recover_c161
            ;;
        --help|-h)
            usage
            ;;
        *)
            log ERROR "未知参数: $1"
            usage
            return 2
            ;;
    esac
}

main "$@"
