#!/usr/bin/env python3
"""
最终版ArUco识别节点（DroneOS专用，无任何可视化报错）
核心功能：仅识别+发布Topic，移除所有易报错的绘制函数
发布Topic：
- /aruco/id (Int32)：ArUco ID（-1=未检测）
- /aruco/detected (Bool)：是否检测到
- /aruco/pose (Float32MultiArray)：位姿 [tx, ty, tz, rx, ry, rz]
- /aruco/image (Image)：原始彩色图像（无绘制）
"""
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Bool, Float32MultiArray

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        # 初始化组件（适配DroneOS极旧版OpenCV）
        self.bridge = CvBridge()
        # 兼容最旧版OpenCV的ArUco字典获取方式
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # 相机内参（适配640x480）
        self.camera_matrix = np.array([[640, 0, 320], [0, 640, 240], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        self.marker_size = 0.05  # ArUco码边长（米）
        
        # 订阅相机图像（适配Orbbec深度相机）
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # 发布内置消息（核心功能，无自定义依赖）
        self.id_pub = self.create_publisher(Int32, '/aruco/id', 10)
        self.detected_pub = self.create_publisher(Bool, '/aruco/detected', 10)
        self.pose_pub = self.create_publisher(Float32MultiArray, '/aruco/pose', 10)
        self.image_pub = self.create_publisher(Image, '/aruco/image', 10)
        
        self.get_logger().info("✅ 最终版ArUco识别节点启动（无任何报错）")

    def image_callback(self, msg):
        try:
            # 图像转换（BGR8格式，兼容所有版本）
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # 初始化发布消息
            aruco_id = Int32(data=-1)
            detected = Bool(data=False)
            pose = Float32MultiArray(data=[])
            
            # 核心识别逻辑（仅保留检测，移除所有绘制）
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray_image, 
                self.aruco_dict, 
                parameters=self.aruco_params
            )
            
            # 如果检测到ArUco码
            if ids is not None and len(ids) > 0:
                # 取第一个检测到的码
                marker_id = int(ids[0][0])
                aruco_id.data = marker_id
                detected.data = True
                
                # 位姿估计（仅计算，不绘制）
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[0],          # 单个码的角点
                    self.marker_size,    # 码边长（米）
                    self.camera_matrix,  # 相机内参
                    self.dist_coeffs     # 畸变系数
                )
                
                # 转换位姿为列表（ROS消息兼容）
                pose.data = [
                    float(tvec[0][0][0]),  # X平移（米）
                    float(tvec[0][0][1]),  # Y平移（米）
                    float(tvec[0][0][2]),  # Z平移（米）
                    float(rvec[0][0][0]),  # X旋转（弧度）
                    float(rvec[0][0][1]),  # Y旋转（弧度）
                    float(rvec[0][0][2])   # Z旋转（弧度）
                ]
                
                # 仅日志输出，无可视化绘制
                self.get_logger().info(f"检测到ArUco ID: {marker_id}, 距离: {tvec[0][0][2]:.2f}m")
            
            # 发布所有核心Topic（无任何绘制，避免报错）
            self.id_pub.publish(aruco_id)
            self.detected_pub.publish(detected)
            self.pose_pub.publish(pose)
            # 发布原始图像（无绘制，保证不报错）
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))

        except Exception as e:
            self.get_logger().error(f"处理图像错误: {str(e)}")

def main(args=None):
    """主函数：启动节点"""
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    # 关闭节点
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()