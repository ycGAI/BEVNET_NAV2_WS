FROM bevnet_modi:v1

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=foxy

# 更新并安装基础工具
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common

# 添加ROS2 GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -

# 添加ROS2软件源
RUN echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list

# 安装ROS2 Foxy
RUN apt-get update && apt-get install -y \
    ros-foxy-ros-base \
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete

# 安装Python包
RUN pip3 install \
    mcap==1.1.1 \
    mcap-ros2-support==0.5.3

# 初始化rosdep
RUN rosdep init || true && rosdep update

# 创建入口脚本
RUN echo '#!/bin/bash\n\
source /opt/ros/foxy/setup.bash\n\
export PYTHONPATH=/workspace/bevnet:/workspace/bevnet/bevnet:$PYTHONPATH\n\
cd /workspace/bevnet_nav2_ws\n\
exec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

WORKDIR /workspace/bevnet_nav2_ws

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]