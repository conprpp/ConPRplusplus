#!/usr/bin/env bash
set -eo pipefail
source "/opt/ros/$ROS_DISTRO/setup.bash"
source /opt/player_ws/install/setup.bash
mkdir -p "$HOME"
export XDG_RUNTIME_DIR="/tmp/conpr-runtime-$(id -u)"
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"
if [[ "${1:-gui}" == gui ]]; then
    shift || true
    if [[ "$ROS_VERSION" == 1 ]]; then
        exec roslaunch conpr_file_player player.launch root:=/data/ConPRplusplus "$@"
    else
        exec ros2 launch conpr_file_player player.launch.py root:=/data/ConPRplusplus "$@"
    fi
fi
exec "$@"
