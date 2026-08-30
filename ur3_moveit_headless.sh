#!/usr/bin/env bash
# Single-command guarded startup for UR3 CB3 + MoveIt + RViz.

TRACER_WORKSPACE="${TRACER_WS:-/home/jt001/tracer_ws}"
CONTROL_LOCK="${TRACER_UR3_CONTROL_LOCK:-/tmp/tracer-ur3-control.lock}"
CALLER_MASTER_URI="${ROS_MASTER_URI:-}"
CALLER_ROS_IP="${ROS_IP:-}"
CALLER_DISPLAY="${DISPLAY:-}"
CALLER_XAUTHORITY="${XAUTHORITY:-}"

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  printf 'ROS Noetic setup not found: /opt/ros/noetic/setup.bash\n' >&2
  exit 1
fi
if [[ ! -f "${TRACER_WORKSPACE}/devel/setup.bash" ]]; then
  printf 'Workspace setup not found: %s/devel/setup.bash\n' "${TRACER_WORKSPACE}" >&2
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1090
source "${TRACER_WORKSPACE}/devel/setup.bash"
set -euo pipefail

export ROS_MASTER_URI="${CALLER_MASTER_URI:-http://192.168.131.1:11311}"
export ROS_IP="${CALLER_ROS_IP:-192.168.131.1}"
unset ROS_HOSTNAME
export DISPLAY="${CALLER_DISPLAY:-:0}"
if [[ -n "${CALLER_XAUTHORITY}" ]]; then
  export XAUTHORITY="${CALLER_XAUTHORITY}"
elif [[ -f /run/user/1000/gdm/Xauthority ]]; then
  export XAUTHORITY=/run/user/1000/gdm/Xauthority
fi

exec flock --exclusive --nonblock --conflict-exit-code 75 "${CONTROL_LOCK}" \
  rosrun tracer_bringup ur3_headless_moveit.py "$@"
