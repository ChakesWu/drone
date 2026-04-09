# 安装系统级 OpenCV（包含 contrib 组件和 ArUco 模块）
sudo apt update
sudo apt install -y python3-opencv libopencv-contrib-dev

1. 看所有 USB 设备（最关键）
lsusb


一、初始化 ArUco_ws 工作空间
# 1. 创建工作空间目录（固定命名为ArUco_ws）
mkdir -p ~/ArUco_ws/src
cd ~/ArUco_ws/src

# 4. 创建纯Python识别节点包
ros2 pkg create --build-type ament_python aruco_detector_py \
    --dependencies rclpy cv_bridge sensor_msgs aruco_msgs_py



三、编写纯 Python ArUco 识别节点（发布 Topic）
1. 核心识别节点代码
nano ~/ArUco_ws/src/aruco_detector_py/aruco_detector_py/aruco_detector_node.py



四、编译 & 运行（适配 DroneOS）
1. 编译 ArUco_ws 工作空间
# 进入工作空间根目录
cd ~/ArUco_ws

# 编译（纯Python，无需C++编译器）
cd ~/ArUco_ws
colcon build --symlink-install
source install/setup.bash

# 可选：将环境加载加入.bashrc，永久生效
echo "source ~/ArUco_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc


2. 启动节点（DroneOS 环境）
# 第一步：启动DroneOS预装的深度相机
ros2 launch orbbec_camera gemini_e.launch.py

# 第二步：新开终端，启动ArUco识别节点（纯Python）
cd ~/ArUco_ws
source ~/ArUco_ws/install/setup.bash
ros2 run aruco_detector_py aruco_detector_node

