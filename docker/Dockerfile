# 基于已有的bevnet_modi:v1容器，该容器已包含Foxy和BEVNet环境
FROM bevnet_modi:v1

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=foxy

# 安装必要的Python包（用于MCAP支持）
RUN pip3 install \
    mcap==1.1.1 \
    mcap-ros2-support==0.5.3 \
    transforms3d>=0.4.1 \
    pyquaternion>=0.9.9

# 创建工作目录
WORKDIR /workspace/bevnet_nav2_ws

# 创建简化的入口脚本（只使用Foxy）
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Source ROS Foxy environment\n\
source /opt/ros/foxy/setup.bash\n\
\n\
# 设置Python路径\n\
export PYTHONPATH=/workspace/bevnet:/workspace/bevnet/bevnet:$PYTHONPATH\n\
\n\
# 如果工作空间已构建，source它\n\
if [ -f "/workspace/bevnet_nav2_ws/install/setup.bash" ]; then\n\
    source /workspace/bevnet_nav2_ws/install/setup.bash\n\
fi\n\
\n\
# 进入工作目录\n\
cd /workspace/bevnet_nav2_ws\n\
\n\
# 执行传入的命令\n\
exec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

# 设置入口点
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]