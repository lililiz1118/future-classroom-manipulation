#!/usr/bin/env bash
# Quick entrypoint for UR3 zero-gravity freedrive teaching.

TRACER_WORKSPACE="${TRACER_WS:-/home/jt001/tracer_ws/.worktrees/ur3-headless-moveit}"

if [[ -f /opt/ros/noetic/setup.bash ]]; then
  source /opt/ros/noetic/setup.bash
fi

if [[ -f "${TRACER_WORKSPACE}/devel/setup.bash" ]]; then
  source "${TRACER_WORKSPACE}/devel/setup.bash"
fi

exec python3 "${TRACER_WORKSPACE}/src/tracer/tracer_ros/tracer_bringup/scripts/ur3_freedrive.py" "$@"
