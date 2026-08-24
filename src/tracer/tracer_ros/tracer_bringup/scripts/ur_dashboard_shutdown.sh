#!/usr/bin/env bash
# 正常停止程序并关闭 UR3 控制器，避免直接断电导致最后关节位置未保存。

set -uo pipefail

ROBOT_IP="${UR_ROBOT_IP:-192.168.131.3}"
DASHBOARD_PORT="${UR_DASHBOARD_PORT:-29999}"
QUERY_TIMEOUT="${UR_DASHBOARD_QUERY_TIMEOUT:-8}"
LOG_FILE="${UR_DASHBOARD_SHUTDOWN_LOG:-${HOME}/.ros/ur_dashboard_shutdown.log}"

mkdir -p "$(dirname "$LOG_FILE")"

log() {
    local level="$1"
    shift
    printf '[%s] [%s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$level" "$*" | tee -a "$LOG_FILE"
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

main() {
    local program_reply robot_reply safety_reply confirmation reply

    robot_reply="$(query_required robotmode)" || return 1
    safety_reply="$(query_required safetymode)" || return 1
    program_reply="$(query_required programState)" || return 1
    log STATE "关机前: ${robot_reply}; ${safety_reply}; ${program_reply}"

    if [[ ! -t 0 ]]; then
        log ERROR "正常关机必须在交互式终端中确认，不能由后台任务自动执行。"
        return 1
    fi
    printf '确认机械臂任务可以结束并准备关闭控制器，输入 SHUTDOWN：'
    IFS= read -r confirmation
    if [[ "$confirmation" != "SHUTDOWN" ]]; then
        log WARN "确认词不匹配，未发送停止或关机命令。"
        return 2
    fi

    if [[ "$program_reply" == PLAYING* ]]; then
        reply="$(query_required stop)" || return 1
        log ACTION "stop -> ${reply}"
        if [[ "$reply" != *"Stopped"* ]]; then
            log ERROR "程序未确认停止，拒绝继续关机。"
            return 1
        fi
        sleep 1
    fi

    reply="$(query_required shutdown)" || return 1
    log ACTION "shutdown -> ${reply}"
    if [[ "$reply" != *"Shutting down"* ]]; then
        log ERROR "控制器未接受关机命令。"
        return 1
    fi

    log INFO "控制器正在正常关机。请等待控制柜完全关闭后再切断总电源。"
}

main "$@"
