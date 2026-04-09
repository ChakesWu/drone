#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
# 导入QoS相关模块
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class TminiCartographerNode(Node):
    def __init__(self):
        super().__init__('tmini_cartographer_node')
        
        # 1. 配置与雷达匹配的QoS策略（关键修复）
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 2. 订阅雷达/scan话题（使用匹配的QoS）
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        
        # 3. 发布雷达的静态TF（base_link -> laser_frame）
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_timer = self.create_timer(0.1, self.publish_laser_tf)
        
        # 4. 雷达数据接收统计
        self.received_scan_count = 0
        self.invalid_scan_time_count = 0
        
        self.get_logger().info("✅ Tmini Cartographer Node started successfully!")
        self.get_logger().info("ℹ️  Please start T-MINI radar now (ros2 launch ydlidar_ros2_driver Tmini_launch.py)")

    def publish_laser_tf(self):
        """发布激光雷达的静态坐标变换（base_link到laser_frame）"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_frame'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.02
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def scan_callback(self, msg):
        """雷达数据回调：忽略scan_time异常，仅验证核心扫描点"""
        # 空数据校验（核心：确保扫描点存在）
        if not msg.ranges:
            self.get_logger().warn("⚠️ Empty scan received! (no points)")
            return
        
        # 累计接收次数
        self.received_scan_count += 1
        
        # 处理scan_time异常（仅记录，不警告）
        if msg.scan_time <= 0:
            self.invalid_scan_time_count += 1
            freq_str = "unknown (driver issue)"
        else:
            freq_str = f"{1.0/msg.scan_time:.1f}Hz"
        
        # 每接收5帧打印一次info日志（验证核心数据）
        if self.received_scan_count % 5 == 0:
            self.get_logger().info(
                f"📡 Received {self.received_scan_count} valid scans | "
                f"Points: {len(msg.ranges)} | Freq: {freq_str} | "
                f"Min range: {msg.range_min}m | Max range: {msg.range_max}m | "
                f"Invalid scan_time: {self.invalid_scan_time_count}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = TminiCartographerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Node stopped by user")
        # 打印最终统计
        node.get_logger().info(f"""
📊 Final Statistics:
- Total valid scans received: {node.received_scan_count}
- Invalid scan_time frames: {node.invalid_scan_time_count}
- Core data (scan points): {"✅ Normal" if node.received_scan_count > 0 else "❌ None"}
        """)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()