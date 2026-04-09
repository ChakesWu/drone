#!/usr/bin/env python3
"""APM位置数据流诊断工具 - 验证MAVROS是否收到有效位置数据"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from mavros_msgs.msg import State #, EKFStatusReport, GPSRAW
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from mavros_msgs.srv import StreamRate
from tf_transformations import euler_from_quaternion
import math

class PositionMonitor(Node):
    def __init__(self):
        super().__init__('position_monitor')
        self.pose_count = 0
        self.last_pose_time = None
        self.ekf_healthy = False
        self.gps_sats = 0
        self.current_yaw = 0

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
        
        # 订阅关键话题（APM专属）
        # self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        # self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, 10)
        # self.create_subscription(EKFStatusReport, '/mavros/ekf_status', self.ekf_cb, 10)
        # self.create_subscription(GPSRAW, '/mavros/gpsstatus/gps1/raw', self.gps_cb, 10)
        
        # self.timer = self.create_timer(1.0, self.diagnostic_report)
        self.get_logger().info("🔍 位置数据监控启动 | 检查: 连接状态/EKF/GPS/位置流")

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
        
        # if future.result() and future.result().success:
        #     self.get_logger().info(f"✓ 位置流速率已设为 {rate_hz}Hz")
        #     return True
        # else:
        #     self.get_logger().error("✗ 设置流速率失败")
        #     return False

    def state_cb(self, msg):
        # if not hasattr(self, 'connected'):
        self.connected = msg.connected
        self.get_logger().info(f"{'✓' if msg.connected else '✗'} 飞控连接: {msg.connected}")
        if self.connected:
            self.get_logger().info(f"Armed: {msg.armed} Mode: {msg.mode}")

    def pose_cb(self, msg):
        self.pose_count += 1
        self.last_pose_time = time.time()
    
        self.get_logger().info(f"x = {msg.pose.position.x:.3f}, y = {msg.pose.position.y:.3f}, z = {msg.pose.position.z:.3f}")
        if self.pose_count == 1:
            self.get_logger().info(f"✓ 首次收到位置数据! z={msg.pose.position.z:.2f}m (应>0)")

    def imu_cb(self, msg):
        quat = msg.orientation
        roll, pitch, yaw = euler_from_quaternion([
            quat.x, quat.y, quat.z, quat.w
        ])
        self.current_yaw = self.normalize_angle(yaw) * (360/math.pi)
        self.get_logger().info(f"yaw = {self.current_yaw:.3f}")

    def normalize_angle(self, angle):
        """将角度归一化到 [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def ekf_cb(self, msg):
        prev = self.ekf_healthy
        self.ekf_healthy = msg.horizontal_pos_abs and msg.velocity_horiz and msg.pos_vert_abs
        if prev != self.ekf_healthy:
            self.get_logger().info(f"{'✓ EKF健康' if self.ekf_healthy else '✗ EKF异常'} | 水平:{msg.horizontal_pos_abs} 速度:{msg.velocity_horiz} 垂直:{msg.pos_vert_abs}")

    def gps_cb(self, msg):
        if msg.satellites_visible != self.gps_sats:
            self.gps_sats = msg.satellites_visible
            fix = "3D" if msg.fix_type >= 3 else "2D" if msg.fix_type == 2 else "无"
            self.get_logger().info(f"🛰️ GPS: {self.gps_sats}颗星 | Fix: {fix} | HDop: {msg.eph:.1f}")

    def diagnostic_report(self):
        elapsed = time.time() - self.last_pose_time if self.last_pose_time else 999
        status = []
        
        if not hasattr(self, 'connected') or not self.connected:
            status.append("✗ 飞控未连接")
        elif self.pose_count == 0:
            status.append("✗ 无位置数据! 检查: SR1_POSITION=10 + MAVROS插件")
        elif elapsed > 2.0:
            status.append(f"✗ 位置数据超时({elapsed:.1f}s)")
        elif not self.ekf_healthy:
            status.append("✗ EKF未健康")
        elif self.gps_sats < 6 and self.ekf_healthy:  # 室内可忽略
            status.append(f"⚠ GPS弱({self.gps_sats}颗星)，依赖视觉定位")
        else:
            status.append(f"✓ 位置流正常 ({self.pose_count} msgs) | EKF:健康 | GPS:{self.gps_sats}颗")
        
        self.get_logger().info(" | ".join(status))
        if "✓ 位置流正常" in status[0]:
            self.get_logger().info("✅ 位置数据验证通过！可进行下一步操作")

def main():
    rclpy.init()
    node = PositionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()