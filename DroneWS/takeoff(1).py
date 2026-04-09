#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import time

class OffboardController(Node):
    """
    一个简单的 ROS 2 节点，用于演示 PX4 的 OFFBOARD 模式
    实现 起飞 -> 悬停 -> 降落 的流程
    """

    def __init__(self):
        super().__init__('offboard_controller')

        # -- 定义 QoS --
        # MAVROS 的状态和服务通常使用 "transient local" 持久性
        # 这意味着它们会保留最后一个消息/状态，以便新加入的节点可以立即获取
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # -- ROS 2 节点组件 --
        self.current_state = State()  # 用于存储无人机的当前状态
        self.target_pose = PoseStamped() # 目标姿态
        self.takeoff_height = 3.0      # 期望的起飞高度 (米)

        # 订阅 MAVROS 状态
        self.state_sub = self.create_subscription(
            State,
            'mavros/state',
            self.state_callback,
            qos_profile
        )

        # 创建 MAVROS 服务客户端
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, 'mavros/cmd/arming')

        # 创建姿态设定点发布者
        self.pose_pub = self.create_publisher(
            PoseStamped,
            'mavros/setpoint_position/local',
            10  # 正常的 QoS 即可
        )

        # 创建一个定时器，用于持续发送设定点
        # 这是 OFFBOARD 模式的强制要求
        # 频率必须高于 2Hz
        self.setpoint_timer = self.create_timer(
            0.05,  # 50ms -> 20Hz
            self.publish_setpoint
        )

        self.get_logger().info('Offboard Controller 节点已初始化.')

    def state_callback(self, msg):
        """存储当前的飞控状态"""
        self.current_state = msg

    def publish_setpoint(self):
        """
        持续发布目标姿态。
        在切换到 OFFBOARD 模式前，必须先开始发布这个。
        """
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.target_pose.header.frame_id = 'map' # 或者 'local_origin'，取决于你的 MAVROS 设置
        self.pose_pub.publish(self.target_pose)

    async def run_mission(self):
        """执行飞行任务的主逻辑"""
        
        self.get_logger().info('等待 MAVROS 连接飞控 (FCU)...')
        while not self.current_state.connected and rclpy.ok():
            await self.wait_for_message(0.1)
        self.get_logger().info('MAVROS 已连接.')

        # 等待服务可用
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0) and rclpy.ok():
            self.get_logger().info('SetMode 服务不可用, 正在等待...')
        while not self.arm_client.wait_for_service(timeout_sec=1.0) and rclpy.ok():
            self.get_logger().info('Arming 服务不可用, 正在等待...')

        self.get_logger().info('MAVROS 服务已准备就绪.')

        # 1. --- 设置初始起飞点 ---
        # 我们将目标 Z 轴设为起飞高度
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = self.takeoff_height
        # 设定点发布定时器已经在 publish_setpoint() 中持续发送这个目标

        self.get_logger().info(f'设定点已设置为 (0, 0, {self.takeoff_height}). 准备起飞...')

        # 2. --- 切换到 OFFBOARD 模式 ---
        # 在发送模式切换请求前，飞控必须已经在接收设定点（我们的定时器正在做这个）
        mode_req = SetMode.Request()
        mode_req.custom_mode = 'OFFBOARD'
        
        while self.current_state.mode != 'OFFBOARD' and rclpy.ok():
            self.get_logger().info('正在尝试切换到 OFFBOARD 模式...')
            future = self.set_mode_client.call_async(mode_req)
            await self.wait_for_future(future, 1.0)
            if future.result() and future.result().mode_sent:
                self.get_logger().info('OFFBOARD 模式已请求.')
            else:
                self.get_logger().warn('切换 OFFBOARD 失败.')
            await self.wait_for_message(1.0)

        self.get_logger().info('已进入 OFFBOARD 模式.')

        # 3. --- 解锁无人机 ---
        arm_req = CommandBool.Request()
        arm_req.value = True

        while not self.current_state.armed and rclpy.ok():
            self.get_logger().info('正在尝试解锁 (Arming)...')
            future = self.arm_client.call_async(arm_req)
            await self.wait_for_future(future, 1.0)
            if future.result() and future.result().success:
                self.get_logger().info('无人机已解锁!')
            else:
                self.get_logger().warn('解锁失败.')
            await self.wait_for_message(1.0)

        self.get_logger().info('无人机已起飞，将悬停 10 秒...')
        
        # 4. --- 悬停 ---
        # 无人机会自动飞向我们之前设置的 target_pose (0, 0, 3.0)
        # 我们只需要等待
        await self.wait_for_message(10.0)

        # 5. --- 降落 ---
        self.get_logger().info('悬停结束，开始降落...')
        land_mode_req = SetMode.Request()
        land_mode_req.custom_mode = 'AUTO.LAND' # PX4 的标准自动降落模式

        while self.current_state.mode != 'AUTO.LAND' and rclpy.ok():
            future = self.set_mode_client.call_async(land_mode_req)
            await self.wait_for_future(future, 1.0)
            if future.result() and future.result().mode_sent:
                self.get_logger().info('已切换到 AUTO.LAND 模式.')
            else:
                self.get_logger().warn('切换 AUTO.LAND 失败.')
            await self.wait_for_message(1.0)

        # 6. --- 等待无人机降落并自动上锁 ---
        self.get_logger().info('等待无人机降落并自动上锁...')
        while self.current_state.armed and rclpy.ok():
            await self.wait_for_message(1.0)
        
        self.get_logger().info('无人机已降落并上锁。任务完成。')

    # --- 辅助函数 ---

    async def wait_for_message(self, duration_sec):
        """简单的异步等待"""
        await asyncio.sleep(duration_sec)

    async def wait_for_future(self, future, timeout_sec):
        """等待一个 ROS 服务的 future 完成"""
        try:
            await asyncio.wait_for(future, timeout=timeout_sec)
        except asyncio.TimeoutError:
            self.get_logger().warn('服务调用超时')
        except Exception as e:
            self.get_logger().error(f'服务调用失败: {e}')


async def main(args=None):
    rclpy.init(args=args)
    
    # 需要导入 asyncio
    import asyncio
    
    controller = OffboardController()
    
    # 我们需要在一个单独的执行器中 spin 节点，以便回调可以被处理
    # 同时在主线程中运行异步任务
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller)
    
    # 启动执行器线程
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        # 运行异步任务
        await controller.run_mission()
    except (KeyboardInterrupt, asyncio.CancelledError):
        controller.get_logger().info('任务被中断.')
    finally:
        controller.get_logger().info('正在关闭节点...')
        controller.setpoint_timer.cancel() # 停止定时器
        controller.destroy_node()
        executor.shutdown()
        # rclpy.shutdown() # executor.shutdown() 似乎会处理这个

if __name__ == '__main__':
    # 我们需要导入 threading
    import threading
    import asyncio
    
    # 使用 asyncio.run 来启动主异步函数
    asyncio.run(main())