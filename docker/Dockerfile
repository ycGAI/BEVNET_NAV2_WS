# 基于你现有的bevnet容器
FROM bevnet_modi:v1

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV PYTHONPATH=/workspace/bevnet:${PYTHONPATH}

# 安装ROS 2 Humble (如果还没有安装)
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    apt-get update && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update || true

# 安装ROS 2基础包
RUN apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-ros-base \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    || true

# 安装Nav2相关包
RUN apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    || true

# 安装Python依赖
RUN pip3 install --no-cache-dir \
    mcap==1.1.1 \
    mcap-ros2-support==0.5.3 \
    ros2-numpy \
    || true

# 创建工作空间
RUN mkdir -p /opt/ros_ws/src
WORKDIR /opt/ros_ws

# 初始化rosdep（如果需要）
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init || true; \
    fi && \
    rosdep update || true

# 设置entrypoint脚本
RUN echo '#!/bin/bash\n\
source /opt/ros/${ROS_DISTRO}/setup.bash\n\
export PYTHONPATH=/workspace/bevnet:${PYTHONPATH}\n\
if [ -f /opt/ros_ws/install/setup.bash ]; then\n\
    source /opt/ros_ws/install/setup.bash\n\
fi\n\
exec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]