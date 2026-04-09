import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import State, Transition
from unique_identifier_msgs.msg import UUID
from abc import ABC, abstractmethod
import threading
import asyncio 
import time

class LifecycleBase(LifecycleNode, ABC):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.node_name = node_name
        self._period = 100  # ms
        self.is_finished = False
        self.is_canceled = False
        self.is_ending = False
        self._base_timer = None
        self.lifecycle_client = LifecycleClient()

    @abstractmethod
    def param_configure(self):
        pass

    @abstractmethod
    def initialize(self):
        pass

    @abstractmethod
    def execute(self):
        pass

    @abstractmethod
    def end(self):
        pass

    @abstractmethod
    def is_finish(self):
        pass

    def configure(self):
        self.lifecycle_client.change_state(self.node_name, "configure")

    def activate(self):
        self.lifecycle_client.change_state(self.node_name, "activate")

    def deactivate(self):
        self.lifecycle_client.change_state(self.node_name, "deactivate")

    def base_timer_callback(self):
        self.execute()
        if self.is_finish():
            # self.get_logger().info(f'is_finish: is_finished = {self.is_finished} and is_canceled = {self.is_canceled} and is_ending = {self.is_ending}')
            self.reset_timer()
            self.deactivate()
            

    def reset_timer(self):
        if self._base_timer is not None:
            self._base_timer.cancel()
            self._base_timer = None

    def on_configure(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.param_configure()
        self.get_logger().info("Configuring...")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        # self.get_logger().info("on_activate...")
        self.is_finished = False
        self.is_canceled = False
        self.is_ending = False
        self.initialize()
        self._base_timer = self.create_timer(self._period / 1000, self.base_timer_callback)
        self.get_logger().info(f"Activating..., period: {self._period}ms")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating...")
        self.end()
        self.is_ending = True
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up...")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down...")
        return TransitionCallbackReturn.SUCCESS

class LifecycleClient(Node):
    def __init__(self):
        super().__init__('lifecycle_client')
        # 创建服务客户端
        self.client_get_state = self.create_client(GetState, 'get_state')
        self.client_change_state = self.create_client(ChangeState, 'change_state')

    def get_state(self, node_name: str):
        request = GetState.Request()
        service_name = f"{node_name}/get_state"

        # 创建客户端
        client = self.create_client(GetState, service_name)

        if client.wait_for_service(timeout_sec=1.0):
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info(f"Current state of {node_name}: {future.result().current_state.label}")
            else:
                self.get_logger().error(f"Failed to call service get_state for {node_name}")
        else:
            self.get_logger().error(f"Service get_state not available for {node_name}")

    def change_state(self, node_name: str, command: str):
        request = ChangeState.Request()

        # 根据命令设置转换类型
        if command == "configure":
            request.transition.id = 1
        elif command == "clearup":
            request.transition.id = 2
        elif command == "activate":
            request.transition.id = 3
        elif command == "deactivate":
            request.transition.id = 4
        elif command == "shutdown":
            request.transition.id = 6
        else:
            self.get_logger().error(f"Unknown command: {command}")
            return

        service_name = f"{node_name}/change_state"

        client = self.create_client(ChangeState, service_name)

        if client.wait_for_service(timeout_sec=1.0):
            future = client.call_async(request)
            # try:
            #     await future
            #     self.get_logger().info(f"State of {node_name} changed successfully")
            # except Exception as e:
            #     self.get_logger().error(f"Failed to call service change_state for {node_name}: {e}")
        else:
            self.get_logger().error(f"Service change_state not available for {node_name}")

