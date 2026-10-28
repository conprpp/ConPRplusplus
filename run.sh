#!/usr/bin/env bash
# Follow the sourced environment; never silently activate another distribution.
set -eo pipefail
PLAYER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "${ROS_VERSION:-}:${ROS_DISTRO:-}" in
  1:noetic|2:humble|2:jazzy) ;;
  *) echo 'Source a supported ROS environment first (Noetic, Humble, Jazzy), then build.sh.' >&2; exit 2 ;;
esac
ws="${CONPR_PLAYER_WS:-$PLAYER_DIR/workspaces/$ROS_DISTRO}"
if [[ -f "$ws/install/setup.bash" ]]; then
  source "$ws/install/setup.bash"
else
  echo "No build for $ROS_DISTRO at $ws. Run ./build.sh first, or use roslaunch/ros2 launch from your own workspace." >&2
  exit 2
fi
export PYTHONUNBUFFERED=1
if [[ "${1:-}" == exec ]]; then shift; exec "$@"; fi
if [[ $ROS_VERSION == 1 ]]; then exec rosrun conpr_file_player player "$@"; fi
exec ros2 run conpr_file_player player "$@"
