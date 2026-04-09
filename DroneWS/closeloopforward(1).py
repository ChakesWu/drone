#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import time
import math # (### 新增 ###) 导入数学库用于计算距离

# 导入 asyncio 和 threading
import asyncio
import threading

class OffboardController(Node):
    """
    (### 修改 ###)
    使用闭环控制的 Offboard 控制器
    实现: 起飞 -> 向前 1 米 -> 悬停 -> 降落
    通过订阅 /mavros/local_position/pose 来验证是否到达目标点
    """

    def __init__(self):
        super().__init__('offboard_controller_closed_loop')

        # -- 定义 QoS --
        qos_profile_transient_local = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_profile_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # -- ROS 2 节点组件 --
        self.current_state = State()
        self.target_pose = PoseStamped() # 最终会由定时器发送的目标点
        self.current_pose = PoseStamped() # (### 新增 ###) 无人机当前位置
        self.pose_received = False       # (### 新增 ###) 是否已收到位置信息
        
        self.takeoff_height = 3.0      # 起飞高度 (米)
        self.flight_distance = 1.0     # 向前飞行距离 (米)
        self.position_tolerance = 0.3  # (### 新增 ###) 位置容忍度 (30厘米)

        # (### 新增 ###) 创建目标位置变量，用于is_at_position比较
        self.takeoff_target_pose = PoseStamped()
        self.forward_target_pose = PoseStamped()

        # 订阅 MAVROS 状态
        self.state_sub = self.create_subscription(
            State,
            'mavros/state',
            self.state_callback,
            qos_profile_transient_local
        )
        
        # (### 新增 ###) 订阅无人机本地位置
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'mavros/local_position/pose',
            self.pose_callback,
            qos_profile_sensor_data # 传感器数据使用 Best Effort
        )

        # 创建 MAVROS 服务客户端
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, 'mavros/cmd/arming')

        # 创建姿态设定点发布者
        self.pose_pub = self.create_publisher(
            PoseStamped,
            'mavros/setpoint_position/local',
            10
        )

        # 创建一个定时器，用于持续发送设定点 (20Hz)
        self.setpoint_timer = self.create_timer(
            0.05,
            self.publish_setpoint
        )

        self.get_logger().info('闭环 Offboard Controller 节点已初始化.')

    def state_callback(self, msg):
        self.current_state = msg

    # (### 新增 ###)
    def pose_callback(self, msg):
        """存储无人机的当前位置"""
        self.current_pose = msg
        self.pose_received = True

    def publish_setpoint(self):
        """
        持续发布目标姿态。
        """
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.target_pose.header.frame_id = 'map' # 确保与 MAVROS TF 配置一致
        self.pose_pub.publish(self.target_pose)

    # (### 新增 ###)
    def is_at_position(self, target_pose, tolerance):
        """
        检查无人机是否在目标位置的容忍范围内
        """
        if not self.pose_received:
            self.get_logger().warn('尚未收到位置信息，无法检查目标点', throttle_duration_sec=5.0)
            return False
            
        dx = self.current_pose.pose.position.x - target_pose.pose.position.x
        dy = self.current_pose.pose.position.y - target_pose.pose.position.y
        dz = self.current_pose.pose.position.z - target_pose.pose.position.z
        
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        return distance < tolerance


    async def run_mission(self):
        """执行飞行任务的主逻辑"""
        
        self.get_logger().info('等待 MAVROS 连接飞控 (FCU)...')
        while not self.current_state.connected and rclpy.ok():
            await self.wait_for_message(0.1)
        self.get_logger().info('MAVROS 已连接.')

        # (### 新增 ###) 等待无人机位置信息
        self.get_logger().info('等待无人机本地位置 (local position)...')
        while not self.pose_received and rclpy.ok():
            self.get_logger().info('尚未收到位置... 检查 MAVROS 和定位系统.', throttle_duration_sec=5)
            await self.wait_for_message(0.1)
        self.get_logger().info('已收到本地位置.')

        # 等待服务可用
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0) and rclpy.ok():
            self.get_logger().info('SetMode 服务不可用, 正在等待...')
        while not self.arm_client.wait_for_service(timeout_sec=1.0) and rclpy.ok():
            self.get_logger().info('Arming 服务不可用, 正在等待...')

        self.get_logger().info('MAVROS 服务已准备就绪.')

        # 1. --- (### 修改 ###) 设置安全起飞点 ---
        # 将第一个目标点设为无人机 *当前* 的 X, Y 位置，和期望的 Z 高度
        # 这样可以防止无人机在起飞时水平移动
        self.takeoff_target_pose.header.frame_id = 'map'
        self.takeoff_target_pose.pose.position.x = self.current_pose.pose.position.x
        self.takeoff_target_pose.pose.position.y = self.current_pose.pose.position.y
        self.takeoff_target_pose.pose.position.z = self.takeoff_height
        self.takeoff_target_pose.pose.orientation = self.current_pose.pose.orientation # 保持当前朝向
        
        # 将 'self.target_pose' (被定时器发送) 设为这个起飞目标
        self.target_pose = self.takeoff_target_pose
        
        self.get_logger().info(f'起飞目标点设为: (X={self.target_pose.pose.position.x:.2f}, Y={self.target_pose.pose.position.y:.2f}, Z={self.takeoff_height})')
        self.get_logger().info('在切换到 OFFBOARD 模式前，持续发送设定点...')
        
        await self.wait_for_message(2.0) # 确保飞控在切换前收到几条消息

        # 2. --- 切换到 OFFBOARD 模式 ---
        mode_req = SetMode.Request()
        mode_req.custom_mode = 'OFFBOARD'
        
        while self.current_state.mode != 'OFFBOARD' and rclpy.ok():
            self.get_logger().info('正在尝试切换到 OFFBOARD 模式...')
            future = self.set_mode_client.call_async(mode_req)
            await self.wait_for_future(future, 1.0)
            await self.wait_for_message(1.0)

        self.get_logger().info('已进入 OFFBOARD 模式.')

        # 3. --- 解锁无人机 ---
        arm_req = CommandBool.Request()
        arm_req.value = True

        while not self.current_state.armed and rclpy.ok():
            self.get_logger().info('正在尝试解锁 (Arming)...')
            future = self.arm_client.call_async(arm_req)
            await self.wait_for_future(future, 1.0)
            await self.wait_for_message(1.0)

        self.get_logger().info('无人机已解锁!')
        
        # 4. --- (### 修改 ###) 步骤 1: 闭环起飞 ---
        self.get_logger().info(f'正在爬升到 {self.takeoff_height} 米...')
        while rclpy.ok() and not self.is_at_position(self.takeoff_target_pose, self.position_tolerance):
            # 打印当前高度
            self.get_logger().info(f'当前高度: {self.current_pose.pose.position.z:.2f} m', throttle_duration_sec=1.0)
            await self.wait_for_message(0.1) # 10Hz 检查频率
            
        self.get_logger().info('已到达起飞高度.')

        # 5. --- (### 修改 ###) 步骤 2: 闭环向前飞行 ---
        self.get_logger().info(f'向前 (X+) 移动 {self.flight_distance} 米...')
        
        # (!!!) 坐标系警告：这里假设 "向前" = 'map' 坐标系的 X 轴正方向
        # 如果需要机体坐标系的 "向前"，则需要进行坐标变换
        
        self.forward_target_pose.header.frame_id = 'map'
        self.forward_target_pose.pose.position.x = self.takeoff_target_pose.pose.position.x + self.flight_distance
        self.forward_target_pose.pose.position.y = self.takeoff_target_pose.pose.position.y
        self.forward_target_pose.pose.position.z = self.takeoff_target_pose.pose.position.z # 保持高度
        self.forward_target_pose.pose.orientation = self.takeoff_target_pose.pose.orientation # 保持朝向

        # 更新 'self.target_pose'，定时器将开始发送这个新目标
        self.target_pose = self.forward_target_pose
        
        self.get_logger().info(f'目标点设为: (X={self.target_pose.pose.position.x:.2f}, Y={self.target_pose.pose.position.y:.2f}, Z={self.target_pose.pose.position.z:.2f})')

        while rclpy.ok() and not self.is_at_position(self.forward_target_pose, self.position_tolerance):
            pos = self.current_pose.pose.position
            self.get_logger().info(f'当前位置: (X={pos.x:.2f}, Y={pos.y:.2f})', throttle_duration_sec=1.0)
            await self.wait_for_message(0.1) # 10Hz 检查频率

        self.get_logger().info(f'已到达 ({self.flight_distance}, 0) 位置.')


        # 6. --- 【步骤 3: 悬停】 ---
        self.get_logger().info('在新位置悬停 5 秒...')
        # 此时设定点仍然是 forward_target_pose，所以无人机会保持不动
        await self.wait_for_message(5.0)

        # 7. --- 【步骤 4: 降落】 ---
        self.get_logger().info('悬停结束，开始降落...')
        land_mode_req = SetMode.Request()
        land_mode_req.custom_mode = 'AUTO.LAND'

        while self.current_state.mode != 'AUTO.LAND' and rclpy.ok():
            future = self.set_mode_client.call_async(land_mode_req)
            await self.wait_for_future(future, 1.0)
            await self.wait_for_message(1.0)

        # 8. --- 等待无人机降落并自动上锁 ---
        self.get_logger().info('等待无人机降落并自动上锁...')
        while self.current_state.armed and rclpy.ok():
            await self.wait_for_message(1.0)
        
        self.get_logger().info('无人机已降落并上锁。任务完成。')

    # --- 辅助函数 ---
    async def wait_for_message(self, duration_sec):
        await asyncio.sleep(duration_sec)

    async def wait_for_future(self, future, timeout_sec):
        try:
            await asyncio.wait_for(future, timeout=timeout_sec)
        except asyncio.TimeoutError:
            self.get_logger().warn('服务调用超时')
        except Exception as e:
            self.get_logger().error(f'服务调用失败: {e}')


async def main(args=None):
    rclpy.init(args=args)
    
    controller = OffboardController()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller)
    
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        await controller.run_mission()
    except (KeyboardInterrupt, asyncio.CancelledError):
        controller.get_logger().info('任务被中断.')
    finally:
        controller.get_logger().info('正在关闭节点...')
        controller.setpoint_timer.cancel()
        controller.destroy_node()
        executor.shutdown()
        executor_thread.join()

if __name__ == '__main__':
    asyncio.run(main())