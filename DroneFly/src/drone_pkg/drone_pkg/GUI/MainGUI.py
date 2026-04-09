from .CtrlUI import *
from .DataUI import *
from .DisplayUI import *


class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("HG-Drone Controller")
        self.resize(800, 600)

        self.DroneFunc = Func("DroneFunc")
        self.ros_thread = threading.Thread(target=self.spin_node, daemon=True)
        self.ros_thread.start()
        # self.DroneFunc = None
        
        # 主布局
        self.main_layout = QtWidgets.QVBoxLayout(self)
        self.main_layout.setContentsMargins(10, 10, 10, 10)
        self.main_layout.setSpacing(10)
        self.tabs = ['功能','测试','遥控']
        
        # 创建各种控件
        self.create_controls()
        
        # 构建布局
        self.build_layout()

    def spin_node(self):
        """专用线程函数用于执行ROS2 spin"""
        try:
            self.DroneFunc.executor.spin()
        finally:
            self.DroneFunc.executor.shutdown()
            rclpy.shutdown()

    def create_controls(self):
        self.CtrlUI_layout = CtrlUI(self.DroneFunc)
        self.DataUI_layout = DataUI(self.DroneFunc)
        self.DisplayUI_layout = DisplayUI(self.DroneFunc)
        
        
        

    def build_layout(self):
        """构建布局结构"""
        self.ctrl_section = BaseSubLayout(QtCore.Qt.Vertical, show_border = False)
        self.ctrl_section.add_subsection([
            self.CtrlUI_layout,
        ], direction=QtCore.Qt.Vertical)

        
        self.data_section = BaseSubLayout(QtCore.Qt.Horizontal, show_border = False)
        self.data_section.add_subsection([
            self.DataUI_layout,
            self.DisplayUI_layout
        ], direction=QtCore.Qt.Horizontal)

        # test_1 = BaseSubLayout(QtCore.Qt.Horizontal, show_border = False)
        # test_1.add_subsection([
        #     self.ServoPWM_layout,
        #     self.Panel_layout,
        # ], direction=QtCore.Qt.Vertical)
        # test_section = BaseSubLayout(QtCore.Qt.Horizontal, show_border = False)
        # test_section.add_subsection([
        #     self.Motor_layout,
        #     test_1,
        # ], direction=QtCore.Qt.Horizontal)
        
        # self.ctrl_section = BaseSubLayout(QtCore.Qt.Horizontal, show_border = False)
        # # ctrl_section.create_tab_widget()
        # self.ctrl_section.add_tab(self.tabs[0], func_section)
        # self.ctrl_section.add_tab(self.tabs[1], test_section)
        # self.ctrl_section.add_tab(self.tabs[2], self.RemoteCtrl_layout)
        # self.ctrl_section.tab_widget.currentChanged.connect(self.tab_changed)
        
        main_section = BaseSubLayout(QtCore.Qt.Vertical, show_border = False)
        main_section.add_subsection([
            self.ctrl_section,
            self.data_section,
        ], direction=QtCore.Qt.Vertical)

        # 将所有子布局添加到主布局
        self.main_layout.addWidget(main_section)

        # 添加伸缩空间使布局更紧凑
        self.main_layout.addStretch(1)
        pass

    def tab_changed(self, index):
        """内部处理标签页切换"""
        self.ctrl_section.current_tab_index = index
        tab_name = self.ctrl_section.tab_widget.tabText(index)
        if tab_name == self.tabs[0]:
            motor_tab = self.Motor_layout.motor_section.get_current_tab()
            if motor_tab['index'] == 1:
                self.Motor_layout.motor_section.switch_to_tab_by_index(0)
        if tab_name == self.tabs[2]:
            self.RemoteCtrl_layout.start_timer_signal.emit()  # 启动定时器信号
            self.RemoteCtrl_layout.lidar_display.start_timer_signal.emit()  # 启动定时器信号
        else:
            self.RemoteCtrl_layout.stop_timer_signal.emit()  # 停止定时器信号
            self.RemoteCtrl_layout.lidar_display.stop_timer_signal.emit()  # 停止定时器信号

        print(f"切换: {tab_name} (索引: {index})")


def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()

# ros2 run application_layer MainGUI