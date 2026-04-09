import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_srvs.srv import Trigger
import math
import time

class DroneAutonomousAPM(Node):
    def __init__(self):
        super().__init__('drone_autonomous_apm')
        
        # ===== APM专用配置 =====
        self.declare_parameter('takeoff_height', 1.0)  # APM起飞高度
        self.declare_parameter('acceptance_radius', 0.3)  # APM航点到达阈值
        self.takeoff_height = self.get_parameter('takeoff_height').value
        self.acceptance_radius = self.get_parameter('acceptance_radius').value
        self.hover_time = 1.0  # 航点悬停时间(秒)
        
        # ===== 状态管理 =====
        self.current_state = State()
        self.current_pose = None
        self.waypoint_index = 0
        self.reached_time = None
        self.last_service_call = 0.0
        self.service_retry_count = 0
        self.max_retries = 3
        
        # ===== APM预设航点 (x, y, z) - ENU坐标系，单位：米 =====
        self.waypoints = [
            (1.0, 0.0, self.takeoff_height),  # 起飞后第一个航点
            (1.0, 1.0, self.takeoff_height),
            (0.0, 1.0, self.takeoff_height)
        ]
        self.target_pose = [0.0, 0.0, 0.0]  # 当前目标位置
        
        # ===== 订阅与发布 =====
        latching_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_cb, latching_qos)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos_profile_sensor_data)
        
        self.pos_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', qos_profile_sensor_data)
        
        # ===== APM专用服务客户端 =====
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')  # APM专用起飞服务
        
        # ===== 定时器 (50Hz) =====
        self.timer = self.create_timer(0.1, self.state_machine_callback)
        self.get_logger().info("APM自主飞行节点已启动 | 飞控类型: ArduPilot")

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg.pose
        self.get_logger().info(f"x = {msg.pose.position.x:.2f}, y = {msg.pose.position.y:.2f}, z = {msg.pose.position.z:.2f}, ")

    def distance_to_target(self):
        """计算当前位置到目标点的欧氏距离 (APM专用)"""
        if not self.current_pose:
            return 1000.0
        dx = self.current_pose.position.x - self.target_pose[0]
        dy = self.current_pose.position.y - self.target_pose[1]
        dz = self.current_pose.position.z - self.target_pose[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def send_setpoint(self, x, y, z):
        """APM专用目标点发送 (无需预发点)"""
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = x, y, z
        # APM中保持当前朝向（可扩展为航向控制）
        if self.current_pose:
            sp.pose.orientation = self.current_pose.orientation
        self.pos_pub.publish(sp)
        self.target_pose = [x, y, z]

    def call_service_safely(self, client, request, service_name):
        """安全调用服务 (APM专用)"""
        now = time.time()
        if now - self.last_service_call < 1.0:  # 1秒冷却
            return False
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"{service_name} 服务不可用")
            return False
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        self.last_service_call = time.time()
        if future.result() and future.result().success:
            self.service_retry_count = 0
            self.get_logger().info(f"✓ {service_name} 成功")
            return True
        else:
            self.service_retry_count += 1
            self.get_logger().error(f"✗ {service_name} 失败 (重试 {self.service_retry_count}/{self.max_retries})")
            return self.service_retry_count < self.max_retries

    def state_machine_callback(self):
        # ===== APM专用状态机逻辑 =====
        if self.current_state.connected:
            # 状态机逻辑 (APM模式)
            if self.current_state.mode == "STABILIZE":  # APM初始模式
                self.get_logger().info("→ 当前模式: STABILIZE (等待解锁)")
                
                # 1. 解锁无人机 (APM专用)
                if not self.current_state.armed:
                    req = CommandBool.Request()
                    req.value = True
                    if self.call_service_safely(self.arming_client, req, "解锁"):
                        self.get_logger().info("✓ 无人机已解锁")
                
                # 2. 切换到GUIDED模式 (APM自主飞行模式)
                elif self.current_state.armed and self.current_state.mode != "GUIDED":
                    req = SetMode.Request()
                    req.custom_mode = "GUIDED"  # APM专用自主模式
                    if self.call_service_safely(self.set_mode_client, req, "切换至GUIDED模式"):
                        self.get_logger().info("✓ 模式已切换至 GUIDED")
            
            # 3. 起飞 (APM专用起飞服务)
            elif self.current_state.mode == "GUIDED" and not self.current_state.armed:
                self.get_logger().info("→ 开始起飞 (GUIDED模式)")
                req = CommandTOL.Request()
                req.altitude = self.takeoff_height  # APM起飞高度
                req.min_pitch = 0.0
                if self.call_service_safely(self.takeoff_client, req, "起飞"):
                    self.get_logger().info(f"→ 起飞高度: {self.takeoff_height}m")
            
            # 4. 航点导航 (APM在GUIDED模式下直接发布目标点)
            elif self.current_state.mode == "GUIDED" and self.current_state.armed:
                if self.waypoint_index < len(self.waypoints):
                    wp = self.waypoints[self.waypoint_index]
                    self.send_setpoint(*wp)  # APM直接发布目标点
                    
                    # 航点到达判断
                    if self.distance_to_target() < self.acceptance_radius:
                        if self.reached_time is None:
                            self.reached_time = time.time()
                            self.get_logger().info(f"→ 到达航点 {self.waypoint_index+1}/{len(self.waypoints)}")
                        elif time.time() - self.reached_time >= self.hover_time:
                            self.waypoint_index += 1
                            self.reached_time = None
                            self.get_logger().info(f"✓ 完成航点 {self.waypoint_index}/{len(self.waypoints)}")
                else:
                    # 5. 降落 (APM专用降落服务)
                    self.get_logger().info("→ 所有航点完成，开始降落")
                    req = CommandTOL.Request()
                    req.altitude = 0.0  # APM降落高度
                    req.min_pitch = 0.0
                    if self.call_service_safely(self.takeoff_client, req, "降落"):
                        self.get_logger().info("✓ 降落指令已发送")
                        self.get_logger().info("★ 任务完成！无人机已安全着陆")
                        self.timer.cancel()  # 停止定时器
        else:
            self.get_logger().warn("⚠ 飞控未连接，等待连接中...")

    def destroy_node(self):
        # APM安全兜底：紧急切换回STABILIZE模式
        try:
            if self.current_state.connected and self.current_state.armed:
                req = SetMode.Request()
                req.custom_mode = "STABILIZE"
                if self.set_mode_client.wait_for_service(timeout_sec=1.0):
                    self.set_mode_client.call_async(req)
                    self.get_logger().warn("⚠ 紧急：尝试切换至 STABILIZE 模式")
        except Exception as e:
            self.get_logger().error(f"安全切换失败: {e}")
        super().destroy_node()

def main():
    rclpy.init()
    node = DroneAutonomousAPM()
    
    try:
        # 等待飞控连接
        while rclpy.ok() and not node.current_state.connected:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not rclpy.ok():
                return
        
        node.get_logger().info("✈️  APM自主飞行系统启动！")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("⚠ 检测到用户中断 (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("系统已安全关闭")

if __name__ == '__main__':
    main()














#!/usr/bin/env python3
"""APM起飞 - 使用CommandTOL服务 + 持续目标点维持"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandTOL
from mavros_msgs.msg import State
import time

class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')
        self.declare_parameter('target_height', 1.0)
        self.target_z = self.get_parameter('target_height').value
        self.current_z = 0.0
        self.armed = False
        self.mode = ""
        self.takeoff_started = False
        
        # 订阅
        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_cb, 10)
        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        
        # 服务
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.timer = self.create_timer(0.05, self.flight_loop)  # 20Hz
        
        self.get_logger().info(f"🛫 准备起飞至 {self.target_z}m | 等待GUIDED模式+已解锁...")

    def state_cb(self, msg):
        self.armed = msg.armed
        self.mode = msg.mode

    def pose_cb(self, msg):
        self.current_z = msg.pose.position.z
        self.get_logger().info(f"x = {msg.pose.position.x:.3f}, y = {msg.pose.position.y:.3f}, z = {msg.pose.position.z:.3f}, ")

    def flight_loop(self):
        # 前置条件检查
        if not (self.armed and self.mode == "GUIDED"):
            if not hasattr(self, 'cond_warn'):
                self.get_logger().warn(f"⏳ 等待: 已解锁=True, 模式=GUIDED (当前: armed={self.armed}, mode={self.mode})")
                self.cond_warn = True
            return
        
        # 起飞服务调用（仅一次）
        if not self.takeoff_started:
            if not self.takeoff_client.wait_for_service(1.0):
                self.get_logger().error("✗ 起飞服务不可用")
                rclpy.shutdown()
                return
            req = CommandTOL.Request()
            req.altitude = self.target_z
            req.min_pitch = 0.0
            future = self.takeoff_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if future.result() and future.result().success:
                self.takeoff_started = True
                self.start_z = self.current_z
                self.get_logger().info(f"↑ 起飞指令已发送 | 目标高度: {self.target_z}m")
            else:
                self.get_logger().error("✗ 起飞服务调用失败")
                rclpy.shutdown()
            return
        
        # 持续发布目标点（维持水平位置+目标高度）
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x = 0.0  # 保持起飞点水平位置
        sp.pose.position.y = 0.0
        sp.pose.position.z = self.target_z
        self.pos_pub.publish(sp)
        
        # 高度监控
        if self.current_z >= self.target_z * 0.9:
            self.get_logger().info(f"✅ 起飞完成! 当前高度: {self.current_z:.2f}m")
            self.get_logger().info("💡 提示: 运行 05_goto_single_point.py 控制水平移动")
            rclpy.shutdown()

def main():
    rclpy.init()
    node = TakeoffNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()