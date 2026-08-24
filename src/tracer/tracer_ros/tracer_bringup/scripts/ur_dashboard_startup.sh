#!/usr/bin/env bash
# UR3 Dashboard 启动序列（带状态校验和详细日志）
# 正常顺序: power_on -> brake_release -> load_program -> play
# 只读诊断: ./ur_dashboard_startup.sh --diagnose
# C161 恢复: ./ur_dashboard_c161_recover.sh --recover-c161

SERVICE_BASE="/ur/ur_hardware_interface/dashboard"
ROBOT_IP="${UR_ROBOT_IP:-192.168.131.3}"
DASHBOARD_PORT="${UR_DASHBOARD_PORT:-29999}"
PROGRAM_FILE="${UR_DASHBOARD_PROGRAM:-ext_ctl.urp}"
WAIT_TIMEOUT="${UR_DASHBOARD_WAIT_TIMEOUT:-20}"
STATE_TIMEOUT="${UR_DASHBOARD_STATE_TIMEOUT:-20}"
CALL_TIMEOUT="${UR_DASHBOARD_CALL_TIMEOUT:-30}"
POLL_INTERVAL="${UR_DASHBOARD_POLL_INTERVAL:-0.5}"
LOG_FILE="${UR_DASHBOARD_LOG_FILE:-${HOME}/.ros/ur_dashboard_startup.log}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RECOVERY_SCRIPT="${SCRIPT_DIR}/ur_dashboard_c161_recover.sh"

# setup.bash 可能写入构建时缓存的 ROS 网络值；优先保留调用者显式传入的值。
CALLER_ROS_MASTER_URI="${ROS_MASTER_URI:-}"
CALLER_ROS_IP="${ROS_IP:-}"

# 直接从普通终端运行时，确保 ROS 命令和工作空间可用。
if [[ -f /opt/ros/noetic/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
fi
if [[ -f /home/jt001/tracer_ws/devel/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /home/jt001/tracer_ws/devel/setup.bash
fi

# ROS 的 setup.bash 在加载过程中会读取尚未定义的变量；加载完成后再启用严格模式。
set -uo pipefail

# 仅在调用者未设置时使用本机 UR 专网默认值。
export ROS_MASTER_URI="${CALLER_ROS_MASTER_URI:-http://192.168.131.1:11311}"
export ROS_IP="${CALLER_ROS_IP:-192.168.131.1}"
unset CALLER_ROS_MASTER_URI CALLER_ROS_IP

mkdir -p "$(dirname "$LOG_FILE")"

log() {
    local level="$1"
    shift
    printf '[%s] [%s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$level" "$*" | tee -a "$LOG_FILE"
}

log_block() {
    local level="$1"
    local text="$2"
    if [[ -z "$text" ]]; then
        log "$level" "  <无输出>"
        return
    fi
    while IFS= read -r line; do
        log "$level" "  $line"
    done <<< "$text"
}

on_interrupt() {
    log WARN "收到中断信号，启动序列已停止；未执行后续步骤。"
    exit 130
}
trap on_interrupt INT TERM

usage() {
    cat <<'EOF'
用法:
  ./ur_dashboard_startup.sh             执行上电启动序列
  ./ur_dashboard_startup.sh --diagnose  只读检查 ROS、服务和机器人状态
  ./ur_dashboard_startup.sh --help      显示帮助

C161 恢复（必须人工确认）:
  ./ur_dashboard_c161_recover.sh --recover-c161

可选环境变量:
  UR_DASHBOARD_PROGRAM             默认 ext_ctl.urp
  UR_ROBOT_IP                     默认 192.168.131.3
  UR_DASHBOARD_PORT               默认 29999
  UR_DASHBOARD_WAIT_TIMEOUT        等待服务秒数，默认 20
  UR_DASHBOARD_STATE_TIMEOUT       等待状态秒数，默认 20
  UR_DASHBOARD_CALL_TIMEOUT        单次服务调用秒数，默认 30
  UR_DASHBOARD_LOG_FILE            默认 ~/.ros/ur_dashboard_startup.log
EOF
}

run_capture() {
    local output
    local rc
    output="$(timeout "$CALL_TIMEOUT" "$@" 2>&1)"
    rc=$?
    printf '%s\n%s' "$rc" "$output"
}

check_master() {
    local result rc output

    log INFO "主机: $(hostname)"
    log INFO "ROS_MASTER_URI=${ROS_MASTER_URI}"
    log INFO "ROS_IP=${ROS_IP}"
    log INFO "ROS_HOSTNAME=${ROS_HOSTNAME:-<未设置>}"
    log INFO "UR Dashboard=${ROBOT_IP}:${DASHBOARD_PORT}"
    log INFO "日志文件: ${LOG_FILE}"

    result="$(run_capture rosnode list)"
    rc="${result%%$'\n'*}"
    output="${result#*$'\n'}"
    if [[ "$rc" -ne 0 ]]; then
        log ERROR "无法连接 ROS master。"
        log_block ERROR "$output"
        log ERROR "请先启动 roscore/底盘 launch，并检查 ROS_MASTER_URI。"
        return 1
    fi

    log INFO "ROS master 可达；当前节点数: $(printf '%s\n' "$output" | sed '/^$/d' | wc -l)"
    if printf '%s\n' "$output" | grep -Fxq "/ur/ur_hardware_interface"; then
        log INFO "UR 驱动节点 /ur/ur_hardware_interface 已注册。"
    else
        log WARN "未发现 UR 驱动节点 /ur/ur_hardware_interface。"
    fi
}

service_exists() {
    local svc="$1"
    local services
    services="$(timeout 5 rosservice list 2>/dev/null || true)"
    printf '%s\n' "$services" | grep -Fxq "$svc"
}

direct_dashboard_query() {
    local command="$1"
    timeout 5 python3 - "$ROBOT_IP" "$DASHBOARD_PORT" "$command" <<'PY'
import socket
import sys

host, port, command = sys.argv[1], int(sys.argv[2]), sys.argv[3]
with socket.create_connection((host, port), timeout=2.0) as sock:
    sock.settimeout(2.0)
    sock.recv(4096)  # Dashboard greeting
    sock.sendall((command + "\n").encode("ascii"))
    print(sock.recv(4096).decode("utf-8", errors="replace").strip())
PY
}

direct_snapshot() {
    local label="$1"
    local robot safety program error

    if ! robot="$(direct_dashboard_query robotmode 2>&1)"; then
        error="$robot"
        log WARN "无法直连 UR Dashboard ${ROBOT_IP}:${DASHBOARD_PORT}。"
        log_block WARN "$error"
        return 1
    fi
    safety="$(direct_dashboard_query safetymode 2>&1 || true)"
    program="$(direct_dashboard_query programState 2>&1 || true)"
    log STATE "${label}: ${robot:-robotmode=UNAVAILABLE}; ${safety:-safetymode=UNAVAILABLE}; ${program:-programState=UNAVAILABLE}"
}

show_ur_nodes_and_services() {
    local nodes services
    nodes="$(timeout 5 rosnode list 2>&1 || true)"
    services="$(timeout 5 rosservice list 2>&1 || true)"

    log INFO "当前 /ur 节点:"
    log_block INFO "$(printf '%s\n' "$nodes" | grep '^/ur/' || true)"
    log INFO "当前 Dashboard 服务:"
    log_block INFO "$(printf '%s\n' "$services" | grep "^${SERVICE_BASE}/" || true)"
}

wait_service() {
    local svc="$1"
    local deadline=$((SECONDS + WAIT_TIMEOUT))
    local next_report=$SECONDS

    log INFO "等待服务 ${svc}（最多 ${WAIT_TIMEOUT}s）..."
    while (( SECONDS < deadline )); do
        if service_exists "$svc"; then
            log INFO "服务已就绪: ${svc}"
            return 0
        fi
        if (( SECONDS >= next_report )); then
            log WARN "服务仍未出现；请确认 tracer_ur_bringup.launch 正在运行。"
            next_report=$((SECONDS + 5))
        fi
        sleep "$POLL_INTERVAL"
    done

    log ERROR "等待服务超时: ${svc}"
    show_ur_nodes_and_services
    direct_snapshot "控制器直连状态" || true
    log ERROR "请先运行: roslaunch tracer_bringup tracer_ur_bringup.launch"
    return 1
}

call_svc() {
    local svc="$1"
    local args="$2"
    local desc="$3"
    local result rc output

    if ! wait_service "$svc"; then
        return 1
    fi

    log INFO "调用 ${desc}: ${svc}"
    result="$(run_capture rosservice call "$svc" "$args")"
    rc="${result%%$'\n'*}"
    output="${result#*$'\n'}"
    log_block INFO "$output"

    if [[ "$rc" -ne 0 ]]; then
        log ERROR "${desc} 调用失败，退出码 ${rc}。"
        return 1
    fi
    if printf '%s\n' "$output" | grep -Eq 'success:[[:space:]]*[Ff]alse'; then
        log ERROR "${desc} 被 Dashboard 拒绝；不会执行后续步骤。"
        return 1
    fi

    log INFO "${desc} 请求已被 Dashboard 接受。"
}

query_answer() {
    local svc="$1"
    local output
    output="$(timeout 5 rosservice call "$svc" 2>/dev/null || true)"
    printf '%s\n' "$output" | sed -n 's/^[[:space:]]*answer: "\(.*\)"/\1/p' | head -n 1
}

query_robot_mode() {
    local answer
    answer="$(query_answer "${SERVICE_BASE}/get_robot_mode")"
    printf '%s\n' "$answer" | sed -n 's/^Robotmode: //p'
}

query_safety_mode() {
    local answer
    answer="$(query_answer "${SERVICE_BASE}/get_safety_mode")"
    printf '%s\n' "$answer" | sed -n 's/^Safetymode: //p'
}

query_program_state() {
    query_answer "${SERVICE_BASE}/program_state"
}

snapshot() {
    local label="$1"
    local robot safety program
    robot="$(query_robot_mode)"
    safety="$(query_safety_mode)"
    program="$(query_program_state)"
    log STATE "${label}: robot=${robot:-UNAVAILABLE}, safety=${safety:-UNAVAILABLE}, program=${program:-UNAVAILABLE}"
}

safety_is_acceptable() {
    local safety="$1"
    [[ "$safety" == "NORMAL" || "$safety" == "REDUCED" ]]
}

wait_robot_state() {
    local stage="$1"
    local expected_regex="$2"
    local deadline=$((SECONDS + STATE_TIMEOUT))
    local robot safety last=""

    log INFO "${stage}: 等待 robot 匹配 ${expected_regex}，最多 ${STATE_TIMEOUT}s。"
    while (( SECONDS < deadline )); do
        robot="$(query_robot_mode)"
        safety="$(query_safety_mode)"

        if [[ "${robot}|${safety}" != "$last" ]]; then
            log STATE "${stage}: robot=${robot:-UNAVAILABLE}, safety=${safety:-UNAVAILABLE}"
            last="${robot}|${safety}"
        fi

        if [[ -n "$safety" ]] && ! safety_is_acceptable "$safety"; then
            log ERROR "检测到非正常安全状态: ${safety}。"
            if [[ "$safety" == "PROTECTIVE_STOP" ]]; then
                log ERROR "可能存在 C161 等保护事件；脚本不会自动 unlock protective stop。"
                log ERROR "若现场确认是 C161 且编码器姿态正确，请运行: ${RECOVERY_SCRIPT} --recover-c161"
            fi
            return 1
        fi

        if [[ "$robot" =~ $expected_regex ]] && safety_is_acceptable "$safety"; then
            log INFO "${stage} 状态确认通过。"
            return 0
        fi
        sleep "$POLL_INTERVAL"
    done

    log ERROR "${stage} 状态确认超时。"
    snapshot "超时状态"
    return 1
}

wait_program_playing() {
    local deadline=$((SECONDS + STATE_TIMEOUT))
    local state safety last=""

    log INFO "等待程序进入 PLAYING，最多 ${STATE_TIMEOUT}s。"
    while (( SECONDS < deadline )); do
        state="$(query_program_state)"
        safety="$(query_safety_mode)"
        if [[ "${state}|${safety}" != "$last" ]]; then
            log STATE "程序确认: program=${state:-UNAVAILABLE}, safety=${safety:-UNAVAILABLE}"
            last="${state}|${safety}"
        fi
        if [[ -n "$safety" ]] && ! safety_is_acceptable "$safety"; then
            log ERROR "程序启动期间进入非正常安全状态: ${safety}。"
            return 1
        fi
        if [[ "$state" == PLAYING* ]] && safety_is_acceptable "$safety"; then
            log INFO "程序已进入 PLAYING。"
            return 0
        fi
        sleep "$POLL_INTERVAL"
    done

    log ERROR "程序未在规定时间内进入 PLAYING。"
    snapshot "程序启动超时"
    return 1
}

diagnose() {
    log INFO "执行只读诊断；不会上电、释放刹车、加载或运行程序。"
    check_master || return 1
    show_ur_nodes_and_services
    if service_exists "${SERVICE_BASE}/get_robot_mode"; then
        snapshot "当前状态"
    else
        log ERROR "Dashboard 状态服务不存在，UR 驱动尚未启动或连接到了不同 ROS master。"
        direct_snapshot "控制器直连状态" || true
        return 1
    fi
}

main() {
    local mode="${1:-start}"
    case "$mode" in
        --diagnose)
            diagnose
            return $?
            ;;
        --help|-h)
            usage
            return 0
            ;;
        start)
            ;;
        *)
            log ERROR "未知参数: ${mode}"
            usage
            return 2
            ;;
    esac

    log INFO "开始 UR3 Dashboard 启动序列。"
    log INFO "目标程序: ${PROGRAM_FILE}"
    check_master || return 1

    wait_service "${SERVICE_BASE}/power_on" || return 1
    wait_service "${SERVICE_BASE}/get_robot_mode" || return 1
    wait_service "${SERVICE_BASE}/get_safety_mode" || return 1
    snapshot "初始状态"

    call_svc "${SERVICE_BASE}/power_on" "{}" "上电" || return 1
    wait_robot_state "上电后" '^(POWER_ON|IDLE|RUNNING)$' || return 1

    call_svc "${SERVICE_BASE}/brake_release" "{}" "释放刹车" || return 1
    wait_robot_state "释放刹车后" '^RUNNING$' || return 1

    call_svc "${SERVICE_BASE}/load_program" "filename: '${PROGRAM_FILE}'" "加载 ${PROGRAM_FILE}" || return 1
    snapshot "加载程序后"

    call_svc "${SERVICE_BASE}/play" "{}" "运行程序" || return 1
    wait_program_playing || return 1

    snapshot "最终状态"
    log INFO "UR3 启动序列完成；Dashboard、机器人和程序状态均已确认。"
}

main "$@"
exit $?
