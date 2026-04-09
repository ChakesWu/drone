#!/usr/bin/env python3
"""APM起飞 - 使用CommandTOL服务 + 持续目标点维持"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State #, EKFStatusReport, GPSRAW
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data
from mavros_msgs.srv import StreamRate
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math

class AutoFly(Node):
    def __init__(self):
        super().__init__('auto_fly')
        self.connected = False
        self.current_state = State()
        self.current_pose = None
        self.current_yaw = 0
        self.step = 0
        self.takeoff_started = False
        self.target_h = 0.45

        self.timer = self.create_timer(0.1, self.timer_cb)

        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_profile_sensor_data)

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
        self.imu_sub = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_cb,
            qos
        )

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        self.stream_rate_client = self.create_client(StreamRate, '/mavros/set_stream_rate')
        self.set_stream_rate(id = 6, rate_hz = 10)  # id = 6 是local_position/pose
        self.set_stream_rate(id = 10, rate_hz = 10)  # id = 10 是/mavros/imu/data

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

    def state_cb(self, msg):
        self.current_state = msg
        self.connected = msg.connected
        # if self.connected:
        #     self.get_logger().info(f"Armed: {msg.armed} Mode: {msg.mode}")

    def pose_cb(self, msg):
        self.current_pose = msg.pose
        # self.get_logger().info(f"x = {msg.pose.position.x:.3f}, y = {msg.pose.position.y:.3f}, z = {msg.pose.position.z:.3f}, ")

    def imu_cb(self, msg):
        quat = msg.orientation
        roll, pitch, yaw = euler_from_quaternion([
            quat.x, quat.y, quat.z, quat.w
        ])
        self.current_yaw = self.normalize_angle(yaw) * (360/math.pi)
        # self.get_logger().info(f"yaw = {self.current_yaw:.3f}")

    def normalize_angle(self, angle):
        """将角度归一化到 [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    
    def arming(self, is_arming):
        while rclpy.ok() and self.arming_client.wait_for_service(1)==False:
            self.get_logger().info(f"等待服务端上线....")
        request = CommandBool.Request()
        request.value = is_arming
        self.arming_client.call_async(request).add_done_callback(self.arming_callback_)
        
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
        self.takeoff_started = True

    def move_to(self, x, y, z, yaw):
        # 无参回调，原发布逻辑完全不变
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = x, y, z

        sp.pose.orientation.x, sp.pose.orientation.y, sp.pose.orientation.z, sp.pose.orientation.w, = quaternion_from_euler(
            0, 0, yaw * (math.pi / 180)
        )
        self.pos_pub.publish(sp)



    def timer_cb(self):
        if self.connected:
            if self.step == 0:
                if not self.current_state.armed:
                    self.setmode("STABILIZE")
                    self.arming(True)
                else:
                    self.get_logger().info("✓ 已解锁")
                    self.step += 1
            elif self.step == 1:
                if self.current_state.mode != "GUIDED":
                    self.setmode("GUIDED")
                else:
                    self.get_logger().info("✓ 模式已切换至 GUIDED")
                    self.step += 1
            elif self.step == 2:
                if not self.takeoff_started and self.current_state.armed:
                    self.takeoff(self.target_h)
                else:
                    # self.get_logger().info("✓ 起飞")
                    self.step += 1
            elif self.step == 3:
                dis_h = math.fabs(self.target_h - self.current_pose.position.z)
                if dis_h < 0.2:
                    self.get_logger().info(f"✓ 起飞完成,当前高度：{self.current_pose.position.z:.3f}")
                    self.step += 1 
                else:
                    self.get_logger().info(f"当前高度：{self.current_pose.position.z:.3f}")
            elif self.step == 4:
                if self.takeoff_started:
                    self.setmode("LAND")
                    self.takeoff_started = False
                else:
                    self.get_logger().info("✓ 降落")
                    self.step += 1
                    
            elif self.step == 5:
                dis_h = math.fabs(self.current_pose.position.z)
                if dis_h < 0.22:
                    self.get_logger().info("✓ 完成")
                    self.arming(False)
                    self.step += 1

        pass


def main():
    rclpy.init()
    node = AutoFly()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.arming(False)
        node.get_logger().warn("⚠ 检测到用户中断 (Ctrl+C)")
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()