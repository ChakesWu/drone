from .BaseSubLayout import *

class CtrlUI(BaseSubLayout):
    def __init__(self,DroneFunc:Func):
        super().__init__(QtCore.Qt.Horizontal, show_border=True)
        self.DroneFunc = DroneFunc

        # 创建各种控件
        self.create_controls()
        
        # 构建布局
        self.build_layout()


    def create_controls(self):
        """创建所有基础控件"""
        self.arming_button = QtWidgets.QPushButton('Arm/Disarm')
        self.arming_button.clicked.connect(self.arming_button_callback)

        self.land_button = QtWidgets.QPushButton('降落')
        self.land_button.clicked.connect(self.land_button_callback)
        
        self.combobox = QtWidgets.QComboBox()
        self.combobox.addItems(["STABILIZE", "POSHOLD", "GUIDED", "LAND"])

        self.setmode_button = QtWidgets.QPushButton('设置模式')
        self.setmode_button.clicked.connect(self.setmode_button_callback)

        self.set_h_input = QtWidgets.QLineEdit()
        self.set_h_input.setPlaceholderText('h')

        self.takeoff_button = QtWidgets.QPushButton('起飞')
        self.takeoff_button.clicked.connect(self.takeoff_button_callback)

        self.set_x_input = QtWidgets.QLineEdit()
        self.set_x_input.setPlaceholderText('x')

        self.set_y_input = QtWidgets.QLineEdit()
        self.set_y_input.setPlaceholderText('y')

        self.set_z_input = QtWidgets.QLineEdit()
        self.set_z_input.setPlaceholderText('z')

        self.set_yaw_input = QtWidgets.QLineEdit()
        self.set_yaw_input.setPlaceholderText('yaw')

        self.send_button = QtWidgets.QPushButton('发送定点')
        self.send_button.clicked.connect(self.send_button_callback)

        self.break_button = QtWidgets.QPushButton('中止')
        self.break_button.clicked.connect(self.break_button_callback)
        

    def build_layout(self):
        """构建布局结构"""

        section_1 = BaseSubLayout(QtCore.Qt.Vertical, show_border=False)
        section_1.add_subsection([
            self.arming_button,
            self.land_button,
        ], direction=QtCore.Qt.Vertical)
        # self.main_layout.addWidget(rotate_section)

        setmode_section = BaseSubLayout(QtCore.Qt.Horizontal, show_border=False)
        setmode_section.add_subsection([
            self.setmode_button, 
            self.combobox, 
        ], direction=QtCore.Qt.Horizontal)
        # self.main_layout.addWidget(pose_input_section)

        takeoff_section = BaseSubLayout(QtCore.Qt.Horizontal, show_border=False)
        takeoff_section.add_subsection([
            self.takeoff_button, 
            self.set_h_input, 
        ], direction=QtCore.Qt.Horizontal)
        
        section_2 = BaseSubLayout(QtCore.Qt.Vertical, show_border = False)
        section_2.add_subsection([
            setmode_section,
            takeoff_section,
        ], direction=QtCore.Qt.Vertical)

        set_pose_section = BaseSubLayout(QtCore.Qt.Horizontal, show_border=False)
        set_pose_section.add_subsection([
            self.set_x_input,
            self.set_y_input,
            self.set_z_input,
            self.set_yaw_input,
        ], direction=QtCore.Qt.Horizontal)
        # self.main_layout.addWidget(set_pose_section)

        pose_button_section = BaseSubLayout(QtCore.Qt.Horizontal, show_border=False)
        pose_button_section.add_subsection([
            self.send_button,
            self.break_button,
        ], direction=QtCore.Qt.Horizontal)
        
        section_3 = BaseSubLayout(QtCore.Qt.Vertical, show_border=False)
        section_3.add_subsection([
            set_pose_section,
            pose_button_section,
        ], direction=QtCore.Qt.Vertical)

        self.add_widgets([
            section_1,
            section_2,
            section_3,
        ])


    def setmode_button_callback(self):
        """按钮点击事件处理"""
        threading.Thread(
            target=self.DroneFunc._drone_node.setmode,
            args=(self.combobox.currentText(),),
            daemon=True
        ).start()


    
    def arming_button_callback(self):
        """按钮点击事件处理"""
        threading.Thread(
            target=self.arming_command,
            # args=(,),
            daemon=True
        ).start()
            

    def arming_command(self):
        """执行升降命令并更新UI状态"""
        armed = self.DroneFunc._drone_node.current_state.armed
        success = self.DroneFunc._drone_node.arming(not armed)
            
    def takeoff_button_callback(self):
        """按钮点击事件处理"""
        h = float(self.set_h_input.text() or 1)
        threading.Thread(
            target=self.DroneFunc._drone_node.takeoff,
            args=(h,),
            daemon=True
        ).start()
        

        
    def land_button_callback(self):
        """按钮点击事件处理"""
        threading.Thread(
            target=self.DroneFunc._drone_node.setmode,
            args=('LAND',),
            daemon=True
        ).start()
            

     
    def send_button_callback(self):
        """按钮点击事件处理"""
        x = float(self.set_x_input.text() or 0)
        y = float(self.set_y_input.text() or 0)
        z = float(self.set_z_input.text() or 1)
        yaw = float(self.set_yaw_input.text() or 0)
        threading.Thread(
            target=self.DroneFunc.drone_moveto,
            args=(x, y, z, yaw, 0.1),
            daemon=True
        ).start()

             
    def break_button_callback(self):
        """按钮点击事件处理"""
        threading.Thread(
            target=self.DroneFunc.break_drone_moveto,
            # args=(,),
            daemon=True
        ).start()



def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)
    controller = Pose()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()

# ros2 run application_layer Pose 