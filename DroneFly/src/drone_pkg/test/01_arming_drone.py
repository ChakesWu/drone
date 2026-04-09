import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ArmingDrone(Node):
    def __init__(self):
        super().__init__('arming_drone')
        self.current_state = State()

        self.timer = self.create_timer(5, self.timer_cb)
        # latching_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            qos
        )

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

    def state_cb(self, msg):
        self.current_state = msg
        self.connected = msg.connected
        # self.get_logger().info(f"{'✓' if msg.connected else '✗'} 飞控连接: {msg.connected}")
        if self.connected:
            self.get_logger().info(f"Armed: {msg.armed} Mode: {msg.mode}")

    def result_callback_(self, result_future):
        response = result_future.result()
        self.get_logger().info(f"收到返回结果：{response.success}")

    def arming(self, is_arming):
        while rclpy.ok() and self.arming_client.wait_for_service(1)==False:
            self.get_logger().info(f"等待服务端上线....")
        request = CommandBool.Request()
        request.value = is_arming
        self.arming_client.call_async(request).add_done_callback(self.result_callback_)

    def timer_cb(self):
        if self.connected:
            if self.current_state.armed == False:
                self.arming(True)
            else:
                self.get_logger().info("arming!")
        else:
            self.get_logger().info("waitting connect")


def main():
    rclpy.init()
    node = ArmingDrone()
    
    try:
        # 等待飞控连接
        while rclpy.ok() and not node.current_state.connected:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not rclpy.ok():
                return
        node.get_logger().info("✈️ APM系统启动！")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("⚠ 检测到用户中断 (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("系统已安全关闭")

if __name__ == '__main__':
    main()




