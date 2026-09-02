#!/usr/bin/env bash
# New-terminal entry point for independently stopping AnyGrasp or UR3.

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd "$(dirname "$SCRIPT_PATH")" && pwd)"
PACKAGE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="$(cd "${PACKAGE_ROOT}/../../../.." && pwd)"

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  printf 'ROS Noetic setup not found: /opt/ros/noetic/setup.bash\n' >&2
  exit 1
fi
if [[ ! -f "${WORKSPACE_ROOT}/devel/setup.bash" ]]; then
  printf 'Workspace setup not found: %s/devel/setup.bash\n' "$WORKSPACE_ROOT" >&2
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1090
source "${WORKSPACE_ROOT}/devel/setup.bash"
set -euo pipefail

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://192.168.131.1:11311}"
export ROS_IP="${ROS_IP:-192.168.131.1}"
unset ROS_HOSTNAME

exec python3 "${SCRIPT_DIR}/robot_stop.py" "$@"
