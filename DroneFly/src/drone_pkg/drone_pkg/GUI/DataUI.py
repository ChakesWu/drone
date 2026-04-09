from .BaseSubLayout import *

class DataWidget(QtWidgets.QWidget):
    def __init__(self, data_name: str, direction=QtCore.Qt.Horizontal):
        super().__init__()
        self.data_name = data_name  # 数据名称

        self.name_label = QtWidgets.QLabel(self.data_name)
        # 创建文本框
        self.text_box = QtWidgets.QLineEdit(self)
        self.text_box.setReadOnly(True)  # 设置为只读

         # 设置文本框大小
        # self.text_box.setFixedSize(50, 30)  # 设置固定大小为 200x100 像素
        # 或者使用 setMinimumSize 和 setMaximumSize
        # self.text_box.setMinimumSize(200, 100)
        # self.text_box.setMaximumSize(300, 150)

        # 布局设置
        self._layout = BaseSubLayout(direction, show_border=False)
        self._layout.add_controls([
            self.name_label,
            self.text_box,
        ])


    def update_text_box(self, data: str):
        """槽函数，用于更新文本框内容"""
        self.text_box.setText(data)  # 更新文本框内容

    def setSize(self, width: int, height: int):
        """设置文本框的大小"""
        self.text_box.set_line_edit_size(width, height)  # 设置固定大小

class DataUI(BaseSubLayout):
    def __init__(self,DroneFunc:Func):
        super().__init__(QtCore.Qt.Vertical, show_border=True, fixed_width=180)
        self.DroneFunc = DroneFunc

        # 创建各种控件
        self.create_controls()
        
        # 构建布局
        self.build_layout()
        
        # 创建并配置定时器
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(100)  # 设置定时间隔为100ms
        self.timer.timeout.connect(self.updateUI)  # 连接定时器的超时信号
        self.start_timer_signal.emit()  # 启动定时器信号

    def create_controls(self):
        """创建所有基础控件"""
        self.x = DataWidget("x: ", QtCore.Qt.Horizontal)
        self.y = DataWidget("y: ", QtCore.Qt.Horizontal)
        self.z = DataWidget("z: ", QtCore.Qt.Horizontal)
        self.roll = DataWidget("roll: ", QtCore.Qt.Horizontal)
        self.pitch = DataWidget("pitch: ", QtCore.Qt.Horizontal)
        self.yaw = DataWidget("yaw: ", QtCore.Qt.Horizontal)

        self.connected = DataWidget("connected: ", QtCore.Qt.Horizontal)
        self.armed = DataWidget("armed:: ", QtCore.Qt.Horizontal)
        self.guided = DataWidget("guided: ", QtCore.Qt.Horizontal)
        self.manual_input = DataWidget("manual_input: ", QtCore.Qt.Horizontal)
        self.mode = DataWidget("mode: ", QtCore.Qt.Horizontal)
        self.system_status = DataWidget("system_status: ", QtCore.Qt.Horizontal)


    def build_layout(self):
        """构建布局结构"""
        pose_section = BaseSubLayout(QtCore.Qt.Vertical, show_border=False)
        pose_section.add_subsection([
            self.x._layout,
            self.y._layout,
            self.z._layout,
            self.roll._layout,
            self.pitch._layout,
            self.yaw._layout,
        ], direction=QtCore.Qt.Vertical)
        state_section = BaseSubLayout(QtCore.Qt.Vertical, show_border=False)
        state_section.add_subsection([
            self.connected._layout,
            self.armed._layout,
            self.mode._layout,
            self.system_status._layout,
            self.guided._layout,
            self.manual_input._layout,
        ], direction=QtCore.Qt.Vertical)


        main_section = BaseSubLayout(QtCore.Qt.Vertical, show_border=False)
        main_section.add_subsection([
            pose_section,
            state_section,
        ], direction=QtCore.Qt.Vertical)
        self.main_layout.addWidget(main_section)


    def updateUI(self):
        """更新反馈状态"""
        cur_x, cur_y, cur_z, cur_roll, cur_pitch, cur_yaw = self.DroneFunc.get_drone_pose()
        state = self.DroneFunc.get_drone_state()
        self.x.update_text_box(f"{cur_x:.2f}")  # 发射信号更新文本框
        self.y.update_text_box(f"{cur_y:.2f}")
        self.z.update_text_box(f"{cur_z:.2f}")
        self.roll.update_text_box(f"{cur_roll:.2f}")
        self.pitch.update_text_box(f"{cur_pitch:.2f}")
        self.yaw.update_text_box(f"{cur_yaw:.2f}")
        self.connected.update_text_box(f"{state.connected}")  # 
        self.armed.update_text_box(f"{state.armed}")
        self.mode.update_text_box(f"{state.mode}")
        self.system_status.update_text_box(f"{state.system_status}")
        self.guided.update_text_box(f"{state.guided}")
        self.manual_input.update_text_box(f"{state.manual_input}")


def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)
    controller = DataUI()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()

# ros2 run application_layer DataUI 