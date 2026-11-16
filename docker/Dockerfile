FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

RUN find /etc/apt -type f \( -name '*.list' -o -name '*.sources' \) \
        -exec sed -i 's|http://|https://|g' {} + \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-simple-commander \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-launch-xml \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-ros-gz \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-rviz2 \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    python3-colcon-common-extensions \
    python3-serial \
    python3-yaml \
    python3-tk \
    espeak-ng \
    alsa-utils \
    python3-pip \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

RUN mkdir -p /ros2_ws/src
RUN mkdir -p /ros2_ws/maps
RUN mkdir -p /ros2_ws/logs

COPY ros2_ws/src /ros2_ws/src
COPY robot_app /ros2_ws/robot_app
COPY code /ros2_ws/code

RUN . /opt/ros/jazzy/setup.bash && \
    cd /ros2_ws && \
    colcon build --symlink-install

COPY docker/entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]
