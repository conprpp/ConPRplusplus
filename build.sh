#!/usr/bin/env bash
set -eo pipefail
PLAYER_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "${ROS_VERSION:-}:${ROS_DISTRO:-}" in
  1:noetic)
    flavor=ros1
    # Native Noetic uses Focal's Python/CMake. An unrelated ML venv earlier on PATH
    # must not silently select its Python or CMake 4 for the catkin build.
    if [[ -f /opt/ros/noetic/setup.bash ]]; then export PATH="/opt/ros/noetic/bin:/usr/bin:/bin:$PATH"; fi
    ;;
  2:humble|2:jazzy) flavor=ros2 ;;
  *) echo 'Source one ROS environment first: Noetic / Humble / Jazzy.' >&2; exit 2 ;;
esac
ws="${CONPR_PLAYER_WS:-$PLAYER_DIR/workspaces/$ROS_DISTRO}"
mkdir -p "$ws"
# A Humble build made in Conda must not be reused silently by native host Humble.
fingerprint="$(python3 -c 'import sys,os,json,importlib; m=importlib.import_module("rospy" if os.environ["ROS_VERSION"]=="1" else "rclpy"); print(json.dumps(dict(python=os.path.realpath(sys.executable),version=sys.version.split()[0],ros=os.path.realpath(m.__file__)),sort_keys=True))')"
if [[ -f "$ws/.conpr_build_environment.json" && "$(cat "$ws/.conpr_build_environment.json")" != "$fingerprint" ]]; then
  echo "This workspace was built with a different Python/ROS installation: $ws" >&2
  echo 'Choose a fresh workspace: CONPR_PLAYER_WS=/path/to/new_workspace ./build.sh' >&2
  exit 2
fi
printf '%s\n' "$fingerprint" > "$ws/.conpr_build_environment.json"
if [[ $flavor == ros1 ]]; then
  mkdir -p "$ws/src"
  for pkg in conpr_file_player livox_ros_driver; do
    target="$PLAYER_DIR/ros1/$pkg"
    if [[ -e "$ws/src/$pkg" && "$(readlink -f "$ws/src/$pkg")" != "$target" ]]; then
      echo "Refusing to replace $ws/src/$pkg" >&2; exit 2
    fi
    ln -sfn "$target" "$ws/src/$pkg"
  done
  catkin_make -C "$ws" -DPYTHON_EXECUTABLE="$(command -v python3)" "$@"
  catkin_make -C "$ws" install
else
  cd "$ws"
  colcon build --base-paths "$PLAYER_DIR/ros2" --build-base "$ws/build" --install-base "$ws/install" \
    --event-handlers console_direct+ "$@" --cmake-args -DPython_EXECUTABLE="$(command -v python3)" -DPython3_EXECUTABLE="$(command -v python3)"
fi
printf '\nBuilt for %s. In this same ROS environment:\n  source "%s/install/setup.bash"\n' "$ROS_DISTRO" "$ws"
printf '  %s/run.sh gui --root /absolute/path/to/ConPRplusplus\n' "$PLAYER_DIR"
