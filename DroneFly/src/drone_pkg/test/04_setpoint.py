import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math

class DroneWaypoint(Node):
    def __init__(self):
        super().__init__('drone_waypoint')
        self.start_pos = None
        self.max_vel = 3.0
        self.target_vel = [0.0, 0.0, 0.0] # vx, vy, vz
        
        # 发布器创建（原逻辑不变）
        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_profile_sensor_data)
        # 定时器创建：0.02s触发一次move_to，核心保留
        self.timer = self.create_timer(0.02, self.move_to)

        #预设航点（原逻辑不变）
        self.waypoints = [(1.0, 0.0, 1.0, 1.0)]
        self.get_logger().info("节点就绪，定时器已启动！")

    def move_to(self):
        # 无参回调，原发布逻辑完全不变
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = 1.0, 0.0, 1.0
        
        sp.pose.orientation.x, sp.pose.orientation.y, sp.pose.orientation.z, sp.pose.orientation.w, = quaternion_from_euler(
            0.0, 0.0, 90.0 * (math.pi / 180)
        )
        self.pos_pub.publish(sp)

def main():
    rclpy.init()
    node = DroneWaypoint()
    # 核心修复：用rclpy.spin(node)替代while+spin_once，让ROS2正常调度定时器
    rclpy.spin(node)
    # 自旋结束后优雅销毁节点
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

