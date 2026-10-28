ARG ROS_IMAGE=ros:jazzy-ros-base-noble
FROM ${ROS_IMAGE}
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-rosdep libgl1-mesa-dri libgl1 libxkbcommon-x11-0 \
    && if [[ "$ROS_VERSION" == 2 ]]; then apt-get install -y python3-colcon-common-extensions; fi \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/conpr_file_player
COPY ros1/ ros1/
COPY ros2/ ros2/
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then rosdep init; fi \
    && rosdep update --include-eol-distros --rosdistro "$ROS_DISTRO" \
    && apt-get update \
    && flavor=ros2 && if [[ "$ROS_VERSION" == 1 ]]; then flavor=ros1; fi \
    && rosdep install --from-paths "$flavor" --ignore-src -y --rosdistro "$ROS_DISTRO" \
    && rm -rf /var/lib/apt/lists/*
COPY conpr_player/ conpr_player/
COPY config/ config/
COPY scripts/ scripts/
COPY build.sh LICENSE ./
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && CONPR_PLAYER_WS=/opt/player_ws bash ./build.sh
COPY docker/entrypoint.sh /usr/local/bin/conpr-entrypoint
RUN chmod 755 /usr/local/bin/conpr-entrypoint
COPY docker/shell-setup.sh /etc/profile.d/conpr-player.sh
COPY docker/create-user.sh /usr/local/lib/conpr-create-user.sh
ARG LOCAL_UID=1000
ARG LOCAL_GID=1000
RUN bash /usr/local/lib/conpr-create-user.sh "$LOCAL_UID" "$LOCAL_GID"
ENV HOME=/home/ubuntu USER=ubuntu LOGNAME=ubuntu
USER ubuntu
WORKDIR /exports
ENTRYPOINT ["/usr/local/bin/conpr-entrypoint"]
CMD ["gui"]
