docker run -it --rm \
    --name bevnet_nav2_container \
    --gpus all \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e PYTHONPATH=/workspace/bevnet:$PYTHONPATH \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/root/.Xauthority:rw \
    -v ~/semantic_bevnet:/workspace/bevnet:rw \
    -v ~/bevnet_nav2_ws:/workspace/bevnet_nav2_ws:rw \
    bevnet_modi:v1 \
    bash



# 设置Python路径
export PYTHONPATH=/workspace/bevnet:$PYTHONPATH

# 进入工作目录
cd /workspace/bevnet_nav2_ws


# 方式1：使用启动脚本
./run_bevnet.sh

# 方式2：直接运行Python脚本
cd src/bevnet_nav2/scripts
python3 bevnet_ros2_node.py --model_path /workspace/bevnet_nav2_ws/models/best.pth.34 --save_output