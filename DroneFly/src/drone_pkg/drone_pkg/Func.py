import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import time
import threading
from geometry_msgs.msg import PoseStamped
import numpy as np
from .app.DroneNode import *
from .app.DroneWaypoint import *


PERIOD_TIME = 0.1

"""使用装饰器来实现单例模式，使得任何类都可以变为单例"""
def singleton(cls):
    instances = {}

    def get_instance(*args, **kwargs):
        if cls not in instances:
            instances[cls] = cls(*args, **kwargs)
        return instances[cls]

    return get_instance

"""该装饰器维护一个字典 instances 来存储类的实例。"""
"""如果类的实例尚不存在，则创建并存储之；如果已存在，则返回现有实例。"""

# @singleton
class Func():
    def __init__(self, name):
        self.name = name
        self.init_finish = False
        self.stop_event = threading.Event()  # 用于信号通知其他线程结束

        self._task_done = False  # 添加标志来确保任务只执行一次

        self._drone_node = DroneNode()
        # self._drone_node = DroneData()
        # self._drone_node = DroneCtrl()
        self._drone_waypoint = DroneWaypoint()
        
        # 创建一个多线程执行器
        self.executor = MultiThreadedExecutor()

        # 添加节点到执行器
        self.executor.add_node(self._drone_node)
        # self.executor.add_node(self._drone_node)
        self.executor.add_node(self._drone_waypoint)


        self.execute_task()

    def SpinExector(self):
        try:
            # 运行执行器
            self.executor.spin()
        finally:
            # 清理资源
            self.executor.shutdown()
        # rclpy.shutdown()
    
    def shutdown(self):
        """安全关闭方法"""
        self.executor.shutdown()

        
    def wait_for_Init(self):
        while(self.init_finish):
            self.init_finish = self._drone_node.is_connected \
                            and self._drone_node.is_connected \
                            and self._drone_waypoint.is_connected \

            time.sleep(PERIOD_TIME)

    
    def execute_task(self):
        # 只有在任务没有执行过的情况下才启动任务
        if not self._task_done:
            self._task_done = True  # 标记任务已经执行
            self.wait_for_Init()
            threading.Thread(target=self.task).start()  # 在新线程中执行任务
        

    def delay_ms(self, t):
        time.sleep(t/1000)
        # return None
        return f"Delay {t}ms"

    def run_in_threads(self, func_with_args):
        """将功能函数与其参数分配到不同的线程并运行，返回结果列表。"""
        results = []  # 存储每个函数的返回值
        threads = []

        def thread_wrapper(func, args):
            result = func(*args)  # 调用函数并获取返回值
            results.append(result)  # 存储返回值（可能是 None）

        for func, args in func_with_args:
            thread = threading.Thread(target=thread_wrapper, args=(func, args))
            threads.append(thread)
            thread.start()  # 启动线程

        # 等待所有线程完成
        for thread in threads:
            thread.join()

        return results
    
    def run_in_order(self, func_with_args):
        """按顺序运行功能函数及其参数，并返回结果列表。"""
        results = []  # 存储每个函数的返回值
        for func, args in func_with_args:
            result = func(*args)  # 解包参数并调用函数
            results.append(result)  # 存储返回值（可能是 None）
        return results
    
    def run_in_threads_with_stop(self, func_with_args):
        """将功能函数与其参数分配到不同的线程并运行，任何一个线程完成后其他线程也停止。"""
        results = []  # 存储每个函数的返回值
        threads = []

        def thread_wrapper(func, args):
            result = func(*args)  # 调用函数并获取返回值
            results.append(result)  # 存储返回值（可能是 None）
            self.stop_event.set()  # 通知其他线程停止

        for func, args in func_with_args:
            thread = threading.Thread(target=thread_wrapper, args=(func, args))
            threads.append(thread)
            thread.start()  # 启动线程

        # 等待任意一个线程完成
        for thread in threads:
            thread.join(timeout=0.1)  # 等待线程完成，设置超时时间

            # 如果 stop_event 被设置，终止其他线程
            if self.stop_event.is_set():
                break

        # 停止所有其他线程
        for thread in threads:
            if thread.is_alive():
                thread.join()  # 等待线程结束

        return results
    
    def PrintResults(self, results):
        for i, result in enumerate(results):
            if result is not None:
                print(f"Result of function {i + 1}: {result}")
            else:
                print(f"Function {i + 1} returned no result.")


    def get_drone_state(self):
        return self._drone_node.current_state

    def get_drone_pose(self):
        return self._drone_node.get_current_pose()


    def drone_arm(self):
        while True:
            if not self._drone_node.current_state.armed:
                self._drone_node.arming(True)
            else:
                break
            time.sleep(PERIOD_TIME)

            
    def drone_disarm(self):
        while True:
            if self._drone_node.current_state.armed:
                self._drone_node.arming(False)
            else:
                break
            time.sleep(PERIOD_TIME)

    def drone_setmode(self, mode):
        while True:
            if self._drone_node.current_state.mode != mode:
                self._drone_node.setmode(mode)
            else:
                break
            time.sleep(PERIOD_TIME)

    def drone_land(self):
        self.drone_setmode('LAND')

    def drone_takeoff(self, h, e = 10):
        self._drone_node.takeoff(float(h) / 100)
        time.sleep(PERIOD_TIME)
        while (self._drone_node.takeoff_flag):
            time.sleep(PERIOD_TIME)
            cur_x, cur_y, cur_z, cur_roll, cur_picth, cur_yaw= self.get_drone_pose()
            dz = math.fabs(h - cur_z)
            if (dz < float(e)):
                self._drone_node.takeoff_flag = False

    def break_drone_takeoff(self):
        self._drone_node.takeoff_flag = False

    def to_global_pose(self, x, y, z, yaw):
        ox = self._drone_node.origin_x
        oy = self._drone_node.origin_y
        oz = self._drone_node.origin_z
        theta = self._drone_node.origin_yaw

        cos_t = np.cos(theta * (math.pi/180))
        sin_t = np.sin(theta * (math.pi/180))
        
        x_global = (ox + x * cos_t - y * sin_t) / 100
        y_global = (oy + x * sin_t + y * cos_t) / 100
        z_global = (oz + z) / 100
        yaw_global = (theta + yaw) * (math.pi/180)
        yaw_global = (yaw_global + np.pi) % (2 * np.pi) - np.pi  # normalize
        return x_global, y_global, z_global, yaw_global

    
    def drone_moveto(self, x, y, z, yaw, e = 10):
        # self._drone_waypoint.waypoint = [x / 100, y / 100, z /100, yaw]
        x_global, y_global, z_global, yaw_global = self.to_global_pose(x, y, z, yaw)
        self._drone_node.get_logger().info(f"x_global = {(x_global * 100):.2f}, y_global = {(y_global * 100):.2f}, z_global = {(z_global * 100):.3f}, yaw_global = {(yaw_global * (180/math.pi)):.2f}, ")
        self._drone_waypoint.waypoint = [x_global, y_global, z_global, yaw_global]
        self._drone_waypoint.activate()
        time.sleep(PERIOD_TIME)
        while (not self._drone_waypoint.is_canceled):
            time.sleep(PERIOD_TIME)
            cur_x, cur_y, cur_z, cur_roll, cur_pitch, cur_yaw= self.get_drone_pose()
            dx = x - cur_x
            dy = y - cur_y
            dz = z - cur_z
            dis = math.sqrt(dx*dx + dy*dy + dz*dz)
            self._drone_node.get_logger().info(f"dis = {dis:.3f}")
            if dis < float(e):
                self._drone_waypoint.is_finished = True
                break

    def break_drone_moveto(self):
        self._drone_waypoint.is_canceled = True
            


    def task(self):
        pass
        # self._drone_node.get_logger().info("start")
        # self.delay_ms(1000)
        # self.drone_arm()
        # self.delay_ms(3000)
        # self.drone_setmode('GUIDED')
        # self.drone_takeoff(0.4, 0.2)
        # self.drone_moveto(0, 0, 0.5, 0, 0.2)
        # self.drone_land()
        # self.delay_ms(1000)
        # self.drone_disarm()

        
def main(args=None):
    rclpy.init(args=args)
    task = Func('app_task')
    try:
        task.SpinExector()
    except KeyboardInterrupt:
        task.drone_disarm()
        task._drone_node.get_logger().warn("⚠ 检测到用户中断 (Ctrl+C)")
    finally:
        task.shutdown()
        rclpy.shutdown()
    


if __name__ == '__main__':
    main()