# Sourced by interactive/login Bash shells inside the image.
if [ -n "${BASH_VERSION:-}" ] && [ -n "${ROS_DISTRO:-}" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    source /opt/player_ws/install/setup.bash
fi
