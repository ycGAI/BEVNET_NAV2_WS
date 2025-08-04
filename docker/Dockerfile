FROM osrf/ros:humble-desktop-full

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=${CUDA_HOME}/bin:${PATH}
ENV LD_LIBRARY_PATH=${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}

# 安装基础依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    git \
    wget \
    vim \
    build-essential \
    cmake \
    libeigen3-dev \
    libboost-all-dev \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

# 安装Python依赖
RUN pip3 install --upgrade pip && \
    pip3 install \
        torch==2.0.1 \
        torchvision==0.15.2 \
        numpy==1.24.3 \
        opencv-python==4.8.0.74 \
        mcap==1.1.1 \
        mcap-ros2-support==0.5.3 \
        scipy \
        pillow \
        pyyaml \
        rospkg \
        transforms3d

# 创建工作空间
RUN mkdir -p /bevnet_nav2_ws/src
WORKDIR /bevnet_nav2_ws

# 安装额外的ROS2包
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-slam-toolbox \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-angles \
    ros-humble-cv-bridge \
    ros-humble-message-filters \
    ros-humble-tf2-sensor-msgs \
    ros-humble-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# 设置entrypoint
COPY docker/entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]