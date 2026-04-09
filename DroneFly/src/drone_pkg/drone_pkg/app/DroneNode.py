import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State #, EKFStatusReport, GPSRAW
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from mavros_msgs.srv import StreamRate
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as np


class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_node')
        self.is_connected = False
        self.takeoff_flag = False
        self.current_state = State()
        self.pose_stamped = PoseStamped()
        self.current_pose = self.pose_stamped.pose
        self.current_x = 0
        self.current_y = 0
        self.current_z = 0
        self.current_roll = 0
        self.current_pitch = 0
        self.current_yaw = 0

        self.origin_x = 0
        self.origin_y = 0
        self.origin_z = 0
        self.origin_roll = 0
        self.origin_pitch = 0
        self.origin_yaw = 0

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_cb,
            qos
        )
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            qos
        )
        # self.imu_sub = self.create_subscription(
        #     Imu,
        #     '/mavros/imu/data',
        #     self.imu_cb,
        #     qos
        # )

        self.stream_rate_client = self.create_client(StreamRate, '/mavros/set_stream_rate')
        
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        # self.is_connected = True



    def set_stream_rate(self, id, rate_hz):
        """强制设置APM位置流速率"""
        if not self.stream_rate_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("✗ 流速率服务不可用")
            return False
            
        req = StreamRate.Request()
        req.stream_id = id  #  消息ID (APM)
        req.message_rate = rate_hz
        req.on_off = True
        
        future = self.stream_rate_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

    def setPoseStream(self, rate = 10):
        self.set_stream_rate(id = 6, rate_hz = rate)  # id = 6 是local_position/pose
    
    def setIMUStream(self, rate = 10):
        self.set_stream_rate(id = 10, rate_hz = rate)   # id = 10 是/mavros/imu/data

        
    def state_cb(self, msg):
        self.current_state = msg
        # self.connected = msg.connected
        self.is_connected = True
        # self.get_logger().info(f"Armed: {msg.armed} Mode: {msg.mode}")

    def pose_cb(self, msg):
        self.current_pose = msg.pose

    
    def normalize_angle(self, angle):
        """将角度归一化到 [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # 获取源位姿，单位m，角度制
    def get_source_pose(self):
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        z = self.current_pose.position.z
        quat = self.current_pose.orientation
        roll, pitch, yaw = euler_from_quaternion([
            quat.x, quat.y, quat.z, quat.w
        ])
        roll = self.normalize_angle(roll) * (180/math.pi)
        pitch = self.normalize_angle(pitch) * (180/math.pi)
        yaw = self.normalize_angle(yaw) * (180/math.pi)
        return x,y,z,roll,pitch,yaw

    # 获取相对与解锁时的位姿，单位cm，角度制
    def get_current_pose(self):
        cos_yaw = np.cos(self.origin_yaw * (math.pi/180))
        sin_yaw = np.sin(self.origin_yaw * (math.pi/180))
        dx = self.current_pose.position.x * 100 - self.origin_x
        dy = self.current_pose.position.y * 100 - self.origin_y
        self.current_z = self.current_pose.position.z * 100 - self.origin_z
        self.current_x = dx * cos_yaw + dy * sin_yaw
        self.current_y = -dx * sin_yaw + dy * cos_yaw

        quat = self.current_pose.orientation
        roll, pitch, yaw = euler_from_quaternion([
            quat.x, quat.y, quat.z, quat.w
        ])
        self.current_roll = self.normalize_angle(roll) * (180/math.pi)
        self.current_pitch = self.normalize_angle(pitch) * (180/math.pi)
        self.current_yaw = self.normalize_angle(yaw - (self.origin_yaw * (math.pi/180))) * (180/math.pi) 
        return self.current_x, self.current_y, self.current_z, self.current_roll, self.current_pitch, self.current_yaw


    # def imu_cb(self, msg):
    #     quat = msg.orientation
    #     roll, pitch, yaw = euler_from_quaternion([
    #         quat.x, quat.y, quat.z, quat.w
    #     ])
    #     self.current_roll = self.normalize_angle(roll) * (180/math.pi)
    #     self.current_pitch = self.normalize_angle(pitch) * (180/math.pi)
    #     self.current_yaw = self.normalize_angle(yaw) * (180/math.pi)
    #     # self.get_logger().info(f"yaw = {self.current_yaw:.3f}")


    def zero_current_pose(self):
        # 纪录解锁时初始位姿
        x,y,z,self.origin_roll,self.origin_pitch,self.origin_yaw = self.get_source_pose()
        self.origin_x = x * 100
        self.origin_y = y * 100
        self.origin_z = z * 100

           
    def arming(self, is_arming):
        while rclpy.ok() and self.arming_client.wait_for_service(1)==False:
            self.get_logger().info(f"等待服务端上线....")
        request = CommandBool.Request()
        request.value = is_arming
        self.arming_client.call_async(request).add_done_callback(self.arming_callback_)
        # 解锁时位姿清零
        if is_arming:
            self.zero_current_pose()
            
        
    def arming_callback_(self, result_future):
        response = result_future.result()
        # self.get_logger().info(f"收到返回结果：{response.success}")

    def setmode(self, mode):
        while rclpy.ok() and self.set_mode_client.wait_for_service(1)==False:
            self.get_logger().info(f"等待服务端上线....")
        request = SetMode.Request()
        request.custom_mode = mode
        self.set_mode_client.call_async(request).add_done_callback(self.setmode_callback_)
        # self.get_logger().info("切换模式")

    def setmode_callback_(self, result_future):
        response = result_future.result()
        # self.get_logger().info(f"收到返回结果：{response}")
        # self.get_logger().info("✓ 模式已切换")

    def takeoff(self, h):
        while rclpy.ok() and self.takeoff_client.wait_for_service(1)==False:
            self.get_logger().info(f"等待服务端上线....")
        request = CommandTOL.Request()
        request.altitude = float(h)
        request.min_pitch = 0.0
        self.takeoff_client.call_async(request).add_done_callback(self.takeoff_callback_)
        self.get_logger().info(f"起飞高度: {h}m")


    def takeoff_callback_(self, result_future):
        response = result_future.result()
        # self.get_logger().info(f"收到返回结果：{response}")
        self.get_logger().info("✓ 起飞")
        self.takeoff_flag = True