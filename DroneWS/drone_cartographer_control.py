import rclpy
import math
import threading
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State

class DroneCartographerNav(Node):
    def __init__(self):
        super().__init__('drone_cartographer_nav')

        # ======================= 订阅 / 发布 =======================
        # TF 监听
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 发布局部位置目标
        self.local_target_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10
        )

        # 飞控状态订阅
        self.current_state = State()
        self.create_subscription(State, '/mavros/state', self.state_cb, 10)

        # 服务客户端
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        # 当前坐标
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # 定时获取位置
        self.create_timer(0.1, self.get_pose_from_tf)

        # 飞行任务线程（不阻塞主程序）
        threading.Thread(target=self.autoflight_task, daemon=True).start()

        self.get_logger().info("✅ 无人机 Cartographer 地图坐标导航已启动！")

    # ======================= 读取无人机当前位置（从TF）=======================
    def get_pose_from_tf(self):
        try:
            # 从 map 到 base_link 转换
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )

            # X Y 坐标
            self.x = transform.transform.translation.x
            self.y = transform.transform.translation.y

            # 四元数 转 Yaw 角
            q = transform.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.yaw = math.atan2(siny_cosp, cosy_cosp) * (180.0 / math.pi)

        except Exception as ex:
            # self.get_logger().warn(str(ex))
            pass

    # ======================= 飞控状态 =======================
    def state_cb(self, msg):
        self.current_state = msg

    # ======================= 发送目标位置 =======================
    def goto_xy(self, target_x, target_y, target_yaw=0.0, height=0.5):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"  # 关键：地图坐标系

        # 位置
        pose.pose.position.x = target_x
        pose.pose.position.y = target_y
        pose.pose.position.z = height

        # 角度（简单版，可自行优化）
        pose.pose.orientation.w = 1.0

        # 发布
        self.local_target_pub.publish(pose)
        self.get_logger().info(
            f"📤 发送目标 → X:{target_x:.2f} Y:{target_y:.2f} | "
            f"当前 → X:{self.x:.2f} Y:{self.y:.2f} Yaw:{self.yaw:.1f}°"
        )

    # ======================= 自动飞行任务 =======================
    def autoflight_task(self):
        import time
        time.sleep(2)

        # 等待连接
        while not self.current_state.connected:
            self.get_logger().info("等待飞控连接...")
            time.sleep(1)

        # 切换 GUIDED 模式
        self.get_logger().info("切换 GUIDED 模式")
        self.set_mode_client.call_async(SetMode.Request(custom_mode="GUIDED"))
        time.sleep(2)

        # 解锁
        self.get_logger().info("解锁电机")
        self.arm_client.call_async(CommandBool.Request(value=True))
        time.sleep(3)

        # 起飞 0.5m
        self.get_logger().info("起飞 0.5m")
        for _ in range(50):
            self.goto_xy(self.x, self.y, height=0.5)
            time.sleep(0.1)

        time.sleep(3)

        # ======================= 地图坐标点飞行 =======================
        self.get_logger().info("🚀 开始地图坐标导航！")

        # 点 1
        for _ in range(100):
            self.goto_xy(-1.0, 0.0)
            time.sleep(0.1)
        time.sleep(5)

        # 点 2
        for _ in range(100):
            self.goto_xy(-1.0, -1.0)
            time.sleep(0.1)
        time.sleep(5)

        # 点3
        for _ in range(100):
            self.goto_xy(0.0, -1.0)
            time.sleep(0.1)
        time.sleep(5)

        # 回到原点
        for _ in range(100):
            self.goto_xy(0.0, 0.0)
            time.sleep(0.1)
        time.sleep(5)

        # 降落
        self.get_logger().info("🛬 自动降落")
        for _ in range(30):
            self.goto_xy(self.x, self.y, height=0.1)
            time.sleep(0.1)

        # 上锁
        self.arm_client.call_async(CommandBool.Request(value=False))
        self.get_logger().info("✅ 任务完成！")

def main(args=None):
    rclpy.init(args=args)
    node = DroneCartographerNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()