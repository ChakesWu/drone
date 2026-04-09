import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandTOL
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math

class DroneTakeoff(Node):
    def __init__(self):
        super().__init__('takeoff')
        self.current_state = State()
        self.current_pose = None
        self.takeoff_height = 0.3
        self.takeoff_flag = False
        self.connected = False

        self.timer = self.create_timer(0.5, self.timer_cb)
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
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_cb,
            qos
        )

        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

    def state_cb(self, msg):
        self.current_state = msg
        self.connected = msg.connected
        # self.get_logger().info(f"{'✓' if msg.connected else '✗'} 飞控连接: {msg.connected}")
        if self.connected:
            self.get_logger().info(f"Armed: {msg.armed} Mode: {msg.mode}")
    
    def pose_cb(self, msg):
        self.current_pose = msg.pose
        # self.get_logger().info(f"x = {msg.pose.position.x:.2f}, y = {msg.pose.position.y:.2f}, z = {msg.pose.position.z:.2f}, ")

    def result_callback_(self, result_future):
        response = result_future.result()
        self.get_logger().info(f"收到返回结果：{response}")
        self.get_logger().info("✓ 起飞")
        self.takeoff_flag = True

    def Takeoff(self, h):
        while rclpy.ok() and self.takeoff_client.wait_for_service(1)==False:
            self.get_logger().info(f"等待服务端上线....")
        request = CommandTOL.Request()
        request.altitude = float(h)
        request.min_pitch = 0.0
        self.takeoff_client.call_async(request).add_done_callback(self.result_callback_)
        self.get_logger().info(f"起飞高度: {self.takeoff_height}m")

    def timer_cb(self):
        if self.connected:
            if self.takeoff_flag == False:
                # self.get_logger().info("The mode is GUIDED")
                self.Takeoff(self.takeoff_height)
            else:
                if self.current_pose != None:
                    dis = math.fabs(self.takeoff_height - self.current_pose.position.z)
                    if dis < 0.2:
                        self.get_logger().info("起飞完成")
                    else:
                        self.get_logger().info(f"当前高度：{self.current_pose.position.z:.2f}")

        else:
            self.get_logger().info("waitting connect")


def main():
    rclpy.init()
    node = DroneTakeoff()
    
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




