###############################################################################
# AGV V0.1 — ROS2 Humble（Windows Docker Desktop）
# 控制：Nav2 → /cmd_vel → agv_control_bridge → TCP 3055 → Mock|真车
###############################################################################
FROM ros:humble-ros-base-jammy

ARG DEBIAN_FRONTEND=noninteractive

# 国内镜像；去掉 deb-src（镜像常无 Sources → apt update 404）
RUN sed -i 's|http://archive.ubuntu.com/ubuntu|http://mirrors.aliyun.com/ubuntu|g' /etc/apt/sources.list && \
    sed -i 's|http://security.ubuntu.com/ubuntu|http://mirrors.aliyun.com/ubuntu|g' /etc/apt/sources.list && \
    sed -i 's|Types: deb deb-src|Types: deb|g' /usr/share/ros-apt-source/ros2.sources && \
    sed -i 's|http://packages.ros.org/ros2/ubuntu|http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' \
      /usr/share/ros-apt-source/ros2.sources && \
    sed -i 's|https://packages.ros.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' \
      /usr/share/ros-apt-source/ros2.sources

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server \
    ros-humble-nav2-amcl \
    ros-humble-nav2-planner \
    ros-humble-nav2-controller \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-costmap-2d \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir -i https://pypi.tuna.tsinghua.edu.cn/simple numpy pyyaml

WORKDIR /ws
RUN mkdir -p /ws/src /ws/maps /opt/agv
COPY scripts/docker_entrypoint.sh /opt/agv/entrypoint.sh
RUN chmod +x /opt/agv/entrypoint.sh

ENV ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    AGV_BACKEND=tcp \
    AGV_HOST=host.docker.internal \
    REAL_ROBOT_ENABLED=false

ENTRYPOINT ["/opt/agv/entrypoint.sh"]
CMD ["bash"]
