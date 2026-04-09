import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class DroneSetMode(Node):
    def __init__(self):
        super().__init__('set_mode')
        self.current_state = State()

        self.timer = self.create_timer(5, self.timer_cb)
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

        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

    def state_cb(self, msg):
        self.current_state = msg
        self.connected = msg.connected
        # self.get_logger().info(f"{'✓' if msg.connected else '✗'} 飞控连接: {msg.connected}")
        if self.connected:
            self.get_logger().info(f"Armed: {msg.armed} Mode: {msg.mode}")

    def result_callback_(self, result_future):
        response = result_future.result()
        self.get_logger().info(f"收到返回结果：{response}")
        self.get_logger().info("✓ 模式已切换")

    def setmode(self, mode):
        while rclpy.ok() and self.set_mode_client.wait_for_service(1)==False:
            self.get_logger().info(f"等待服务端上线....")
        request = SetMode.Request()
        request.custom_mode = mode
        self.set_mode_client.call_async(request).add_done_callback(self.result_callback_)
        self.get_logger().info("切换模式")

    def timer_cb(self):
        if self.connected:
            if self.current_state.mode == "STABILIZE":
                self.get_logger().info("The mode is STABILIZE...")
                self.setmode("GUIDED")
            else:
                self.get_logger().info("The mode is GUIDED!")
        else:
            self.get_logger().info("waitting connect")


def main():
    rclpy.init()
    node = DroneSetMode()
    
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




