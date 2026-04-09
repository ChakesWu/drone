from .Func import *

@singleton
class Task(Func):
    def __init__(self, name):
        super().__init__(name)

    def task(self):
        self._drone_data.get_logger().info("start")
        # self.delay_ms(1000)
        # self.drone_arm()
        # self.delay_ms(3000)
        # self.drone_setmode('GUIDED')
        # self.drone_takeoff(0.4, 0.2)
        self.drone_moveto(0, 0, 50, 0, 0.2)
        # self.drone_land()
        # self.delay_ms(1000)
        # self.drone_disarm()

        
def main(args=None):
    rclpy.init(args=args)
    task = Task('app_task')
    try:
        task.SpinExector()
    except KeyboardInterrupt:
        task.drone_disarm()
        task._drone_data.get_logger().warn("⚠ 检测到用户中断 (Ctrl+C)")
    finally:
        task.shutdown()
        rclpy.shutdown()