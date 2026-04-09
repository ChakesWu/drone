import sys
from PyQt5 import QtWidgets, QtCore, QtGui
import threading
import rclpy
from ..Func import *


class BaseSubLayout(QtWidgets.QWidget):
    start_timer_signal = QtCore.pyqtSignal()
    stop_timer_signal = QtCore.pyqtSignal()
    """增强版布局容器，支持多级嵌套混合布局"""
    def __init__(self, direction=QtCore.Qt.Vertical, title=None, parent=None, show_border=False, fixed_width=None, fixed_height=None):
        super().__init__(parent)
        
        # 创建主容器Frame（用于边框显示）
        if show_border:
            self.frame = QtWidgets.QFrame()
            self.frame.setFrameStyle(QtWidgets.QFrame.Box | QtWidgets.QFrame.Plain)  # 设置边框样式
            self.frame.setLineWidth(1)  # 边框宽度
        else:
            self.frame = QtWidgets.QWidget()  # 不显示边框则使用 QWidget

        # 主布局方向（将frame作为容器）
        self.main_layout = QtWidgets.QVBoxLayout(self.frame) if direction == QtCore.Qt.Vertical else QtWidgets.QHBoxLayout(self.frame)
        self.main_layout.setContentsMargins(5, 5, 5, 5)
        self.main_layout.setSpacing(5)

        # 外层布局管理frame和标题
        outer_layout = QtWidgets.QVBoxLayout(self)
        outer_layout.setContentsMargins(0, 0, 0, 0)
        
        # 添加标题
        if title:
            self.add_title(title, outer_layout)
        
        outer_layout.addWidget(self.frame)

        # 定时器槽
        self.start_timer_signal.connect(self.start_timer)
        self.stop_timer_signal.connect(self.stop_timer)

        # 标签页相关属性
        self.tab_widget = None
        self.current_tab_index = -1
        self.tabs = {}  # 存储标签页的字典 {tab_name: widget}

        # ---------- 硬设定整个控件的固定尺寸 ----------
        if fixed_width is not None:
            self.setFixedWidth(fixed_width)
        if fixed_height is not None:
            self.setFixedHeight(fixed_height)


    def add_title(self, text, parent_layout):
        """添加标题栏（修改后的版本）"""
        title_bar = QtWidgets.QWidget()
        title_layout = QtWidgets.QHBoxLayout(title_bar)
        title_layout.setContentsMargins(0, 0, 0, 0)
        
        label = QtWidgets.QLabel(text)
        label.setStyleSheet("""
            font-weight: bold; 
            color: #2c3e50;
            border-bottom: 1px solid #bdc3c7;
            padding: 2px;
        """)
        title_layout.addWidget(label)
        title_layout.addStretch()
        
        parent_layout.insertWidget(0, title_bar)

     # ---------- 硬设定内部框架固定尺寸的方法 ----------
    def set_frame_fixed_width(self, width):
        """设置内部框架的固定宽度"""
        self.frame.setFixedWidth(width)

    def set_frame_fixed_height(self, height):
        """设置内部框架的固定高度"""
        self.frame.setFixedHeight(height)

    def set_frame_fixed_size(self, width, height):
        """设置内部框架的固定尺寸"""
        self.frame.setFixedSize(width, height)

    
    # ---------- 重载：add_subsection 支持固定尺寸 ----------
    def add_subsection(self, widgets, direction=QtCore.Qt.Horizontal, stretch=0,
                       fixed_width=None, fixed_height=None):   # 新增固定尺寸参数
        """
        添加子区域（支持任意方向），并可强制设定该区域的固定宽度/高度
        """
        container = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(container) if direction == QtCore.Qt.Vertical else QtWidgets.QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        
        for item in widgets:
            if isinstance(item, QtWidgets.QLayout):
                container.layout().addLayout(item)
            elif isinstance(item, QtWidgets.QWidget):
                container.layout().addWidget(item, stretch=stretch)
        
        # ---------- 硬设定子区域固定尺寸 ----------
        if fixed_width is not None:
            container.setFixedWidth(fixed_width)
        if fixed_height is not None:
            container.setFixedHeight(fixed_height)
        
        self.main_layout.addWidget(container)
        return container

    def add_controls(self, widgets, direction=QtCore.Qt.Horizontal, stretch=0):
        """直接添加控件（不创建子容器）"""
        for widget in widgets:
            if isinstance(widget, QtWidgets.QLayout):
                self.main_layout.addLayout(widget)
            else:
                self.main_layout.addWidget(widget, stretch=stretch)


    def add_widgets(self, widgets, stretch=0):
        """添加多个控件到布局"""
        for widget in widgets:
            self.main_layout.addWidget(widget, stretch=stretch)
    
    def add_layout(self, layout):
        """添加嵌套布局"""
        self.main_layout.addLayout(layout)

    def add_grid_subsection(self, widget_matrix):
        """添加网格布局子区域"""
        grid = QtWidgets.QGridLayout()
        for row, widgets in enumerate(widget_matrix):
            for col, widget in enumerate(widgets):
                grid.addWidget(widget, row, col)
        self.main_layout.addLayout(grid)


    def create_tab_widget(self):
        """创建标签页容器（需在添加具体标签页前调用）"""
        if not self.tab_widget:
            self.tab_widget = QtWidgets.QTabWidget()
            self.tab_widget.currentChanged.connect(self._handle_tab_changed)
            self.add_widgets([self.tab_widget])
        return self.tab_widget

    def add_tab(self, name, content_widget=None, layout_direction=QtCore.Qt.Horizontal, fixed_width=None, fixed_height=None):
        """
        添加新标签页（通用方法）
        :param name: 标签页名称
        :param content_widget: 要放入的内容控件（可选）
        :param layout_direction: 内容布局方向（当content_widget为None时自动创建）
        :return: 返回内容控件的容器（QWidget）
        """
        if not self.tab_widget:
            self.create_tab_widget()

        # 创建标签页容器
        tab = QtWidgets.QWidget()
        tab_layout = QtWidgets.QVBoxLayout(tab) if layout_direction == QtCore.Qt.Vertical else QtWidgets.QHBoxLayout(tab)
        tab_layout.setContentsMargins(2, 2, 2, 2)
        
        # 添加内容控件
        if content_widget:
            if isinstance(content_widget, QtWidgets.QLayout):
                tab.setLayout(content_widget)
            else:
                tab_layout.addWidget(content_widget)

        # ---------- 硬设定标签页容器固定尺寸 ----------
        if fixed_width is not None:
            tab.setFixedWidth(fixed_width)
        if fixed_height is not None:
            tab.setFixedHeight(fixed_height)

        self.tab_widget.addTab(tab, name)
        self.tabs[name] = tab
        return tab  # 返回容器便于后续操作

    def get_current_tab(self):
        """获取当前选中的标签页信息"""
        index = self.tab_widget.currentIndex()
        return {
            "index": index,
            "name": self.tab_widget.tabText(index),
            "widget": self.tab_widget.currentWidget()
        }

    def _handle_tab_changed(self, index):
        """内部处理标签页切换"""
        self.current_tab_index = index
        tab_name = self.tab_widget.tabText(index)
        print(f"切换到标签页: {tab_name} (索引: {index})")

    def switch_to_tab_by_index(self, index: int):
        """
        根据索引切换标签页
        :param index: 标签页索引（从0开始）
        """
        if self.tab_widget and 0 <= index < self.tab_widget.count():
            self.tab_widget.setCurrentIndex(index)
        else:
            print(f"无效的标签页索引: {index} (总标签数: {self.tab_widget.count()})")

    def switch_to_tab_by_name(self, name: str):
        """
        根据名称切换标签页（扩展功能）
        :param name: 标签页名称
        """
        if self.tab_widget:
            for i in range(self.tab_widget.count()):
                if self.tab_widget.tabText(i) == name:
                    self.tab_widget.setCurrentIndex(i)
                    return
            print(f"未找到标签页: {name}")


    def start_timer(self):
        """启动定时器"""
        self.timer.start()

    def stop_timer(self):
        """停止定时器"""
        self.timer.stop()

# 调用示例：
# section.add_grid_subsection([
#     [QtWidgets.QLabel("用户名："), QtWidgets.QLineEdit()],
#     [QtWidgets.QLabel("密码："), QtWidgets.QLineEdit()]
# ])


"""边框结构优化："""
# self.frame = QtWidgets.QFrame()
# self.frame.setFrameStyle(QtWidgets.QFrame.Box | QtWidgets.QFrame.Plain)
# outer_layout.addWidget(self.frame)  # 将frame放入外层布局

"""布局层级调整："""
# # 原布局改为frame的内部布局
# self.main_layout = QtWidgets.QVBoxLayout(self.frame)  # 而不是直接放在self上

# # 外层布局管理标题和frame
# outer_layout = QtWidgets.QVBoxLayout(self)
# outer_layout.addWidget(self.frame)

"""标题样式增强："""
# label.setStyleSheet("""
#     border-bottom: 1px solid #bdc3c7;  # 添加下划线分隔
#     padding: 2px;  # 增加内边距
# """)

"""边距优化："""
# outer_layout.setContentsMargins(0, 0, 0, 0)  # 外层布局不留边距
# self.main_layout.setContentsMargins(5, 5, 5, 5)  # 内部内容保持边距

"""自定义边框样式"""
# # 修改边框颜色
# self.frame.setStyleSheet("QFrame { border: 1px solid #e74c3c; }")

# # 修改圆角边框
# self.frame.setStyleSheet("""
#     QFrame {
#         border: 1px solid #bdc3c7;
#         border-radius: 5px;
#     }
# """)

# # 修改标题样式
# label.setStyleSheet("""
#     font-size: 14px;
#     color: #3498db;
#     border-bottom: 2px dashed #3498db;
# """)
