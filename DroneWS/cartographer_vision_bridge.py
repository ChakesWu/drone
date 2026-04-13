#!/usr/bin/env python3
"""
cartographer_vision_bridge.py
==============================
将 Cartographer 的 TF 定位结果（map → base_link）
桥接到 MAVROS 的外部视觉定位接口：
  /mavros/vision_pose/pose  (PoseStamped)

这样飞控 EKF 就能接收到可靠的本地坐标，
使 local_position/pose 不再为空或漂移，
从而让自主飞行（GUIDED 模式）可以正常工作。

【ArduPilot 飞控端配置（必须在 Mission Planner 中设置）】
  AHRS_EKF_TYPE = 3          （启用 EKF3）
  EK3_ENABLE    = 1
  EK3_SRC1_POSXY = 6         （外部导航 ExtNav 作为 XY 定位源）
  EK3_SRC1_POSZ  = 1         （气压计作为 Z 定位源，推荐；或设 6 用视觉 Z）
  EK3_SRC1_YAW   = 6         （外部导航作为 Yaw 源）
  VISO_TYPE      = 1         （启用外部视觉里程计）

【启动顺序】
  1. ros2 launch mavros apm.launch fcu_url:="serial:///dev/ttyACM0:57600"
  2. ros2 launch ydlidar_ros2_driver Tmini_launch.py
  3. ros2 launch tmini_cartographer_py tmini_cartographer.launch.py
  4. python3 cartographer_vision_bridge.py   <-- 本脚本
  5. python3 drone_cartographer_control.py   <-- 飞行任务脚本

【验证桥接是否成功】
  ros2 topic echo /mavros/vision_pose/pose   # 应有持续数据流
  ros2 topic echo /mavros/local_position/pose  # EKF 融合后这里也会有数据
"""

import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PoseStamped


class CartographerVisionBridge(Node):
    def __init__(self):
        super().__init__('cartographer_vision_bridge')

        # ── TF 监听器 ──────────────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── 发布到 MAVROS vision_pose（RELIABLE + VOLATILE，MAVROS 要求）──
        vision_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.vision_pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            vision_qos
        )

        # ── 状态统计 ────────────────────────────────────────────────────────
        self._published_count = 0
        self._fail_count = 0

        # ── 定时器：30 Hz 发布（ArduPilot EKF 建议 >= 10 Hz）──────────────
        self.create_timer(1.0 / 30.0, self._publish_vision_pose)

        self.get_logger().info(
            '✅ Cartographer→MAVROS vision_pose 桥接节点已启动 (30 Hz)\n'
            '   订阅 TF: map → base_link\n'
            '   发布至: /mavros/vision_pose/pose\n'
            '   请确认飞控已设置 EK3_SRC1_POSXY=6, VISO_TYPE=1'
        )

    def _publish_vision_pose(self):
        try:
            # 获取最新 TF（超时 50ms，避免阻塞）
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
        except Exception:
            self._fail_count += 1
            # 前 5 秒内频繁失败属正常（Cartographer 还在初始化），之后才警告
            if self._fail_count > 150:
                self.get_logger().warn(
                    f'⚠️  TF 查询失败 {self._fail_count} 次。'
                    '请确认 Cartographer 已启动且激光雷达有数据。',
                    throttle_duration_sec=5.0
                )
            return

        # ── 构造 PoseStamped 并发布 ──────────────────────────────────────
        pose = PoseStamped()
        # 时间戳使用 TF 本身的时间，避免时间戳不一致
        pose.header.stamp = transform.header.stamp
        pose.header.frame_id = 'map'

        t = transform.transform.translation
        q = transform.transform.rotation

        pose.pose.position.x = t.x
        pose.pose.position.y = t.y
        pose.pose.position.z = t.z

        pose.pose.orientation.x = q.x
        pose.pose.orientation.y = q.y
        pose.pose.orientation.z = q.z
        pose.pose.orientation.w = q.w

        self.vision_pub.publish(pose)
        self._published_count += 1

        # 每 300 帧（约 10 秒）打印一次状态，方便确认运行正常
        if self._published_count % 300 == 0:
            yaw_deg = self._quat_to_yaw_deg(q)
            self.get_logger().info(
                f'📡 已发布 {self._published_count} 帧 | '
                f'X:{t.x:.3f}  Y:{t.y:.3f}  Z:{t.z:.3f}  Yaw:{yaw_deg:.1f}°'
            )

    @staticmethod
    def _quat_to_yaw_deg(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp) * (180.0 / math.pi)


def main(args=None):
    rclpy.init(args=args)
    node = CartographerVisionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('桥接节点已停止。')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
