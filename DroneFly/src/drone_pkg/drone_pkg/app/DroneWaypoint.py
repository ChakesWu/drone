from .GenericClass import *
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math


class DroneWaypoint(LifecycleBase):
    def __init__(self):
        super().__init__('drone_waypoint')
        self.is_connected = False
        self._period = 50
        self.configure()
        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_profile_sensor_data)
        self.waypoint = [1,2,3,180]

        self.is_connected = True

    
    def move_to(self, x, y, z, yaw):
        # 无参回调，原发布逻辑完全不变
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = float(x), float(y), float(z)

        sp.pose.orientation.x, sp.pose.orientation.y, sp.pose.orientation.z, sp.pose.orientation.w, = quaternion_from_euler(
            0, 0, float(yaw) * (math.pi / 180)
        )
        self.pos_pub.publish(sp)

    
    
    def param_configure(self):
        pass

    def initialize(self):
        self.is_finished = False
        self.is_canceled = False

    def execute(self):
        self.move_to(self.waypoint[0], self.waypoint[1], self.waypoint[2], self.waypoint[3])

    def end(self):
        # self.is_finished = False
        # self.is_canceled = False
        self.get_logger().info("END")

    def is_finish(self):
        return self.is_finished or self.is_canceled