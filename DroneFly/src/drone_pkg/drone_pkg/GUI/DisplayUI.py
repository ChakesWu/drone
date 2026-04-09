from .BaseSubLayout import *
import sys
import math
from PyQt5 import QtWidgets, QtCore, QtGui

DEFAULT_SCALE = 2.0       #默认缩放比例

class DisplayCanvas(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.points = []
        self.view_offset = QtCore.QPoint(0, 0)
        self.current_scale = DEFAULT_SCALE     # 初始缩放比例
        self.selection_rect = None
        
        self.dragging = False           # 拖动状态标识
        self.last_mouse_pos = None      # 上次鼠标位置
        self.selection_start = None     # 框选起始坐标

        # 调整网格参数（可选）
        self.grid_max_radius = 800  # 网格最大半径（原为20）
        self.grid_interval = 20     # 网格间隔（保持1米）
        self.point_radius = 5    # 点大小参数（单位：米）
        
        # 显示参数配置
        self.point_color = QtGui.QColor(255, 0, 0, 150)
        self.background_color = QtGui.QColor(32, 32, 32)
        self.grid_color = QtGui.QColor(100, 100, 100, 50)
        self.crosshair_color = QtGui.QColor(200, 200, 200, 100)
        
        # 性能优化设置
        self.setAttribute(QtCore.Qt.WA_OpaquePaintEvent)
        self.setMinimumSize(400, 350)
        self.setMouseTracking(True)

    def paintEvent(self, event):
        """自定义绘制事件"""
        painter = QtGui.QPainter(self)
        try:
            painter.setRenderHints(
                QtGui.QPainter.Antialiasing | 
                QtGui.QPainter.HighQualityAntialiasing |
                QtGui.QPainter.SmoothPixmapTransform
            )
            
            # 绘制背景
            painter.fillRect(event.rect(), self.background_color)
            
            # 坐标变换
            center = self.rect().center() + self.view_offset
            painter.translate(center.x(), center.y())
            painter.scale(self.current_scale, -self.current_scale)
            
            # 绘制网格系统
            self._draw_grid(painter)
            
            # 绘制点云（优化绘制方式）
            if self.points:
                path = QtGui.QPainterPath()
                for x, y in self.points:
                    path.addEllipse(QtCore.QPointF(x, y), 
                                    self.point_radius / self.current_scale,      #X轴半径
                                    self.point_radius / self.current_scale)      #y轴半径
                painter.fillPath(path, self.point_color)
            
            # 绘制交互元素
            self._draw_crosshair(painter)
            self._draw_selection_rect(painter)
        finally:
            painter.end()

    def _draw_grid(self, painter):
        """优化的网格绘制方法"""
        grid_pen = QtGui.QPen(self.grid_color, 0)
        painter.setPen(grid_pen)
        
        grid_path = QtGui.QPainterPath()
        # 使用新的网格参数
        for r in range(1, self.grid_max_radius+1, self.grid_interval):
            grid_path.addEllipse(QtCore.QPointF(0,0), r, r)
        
        # 调整角度线长度匹配网格
        for angle in range(0, 360, 30):
            rad = math.radians(angle)
            x = self.grid_max_radius * math.cos(rad)  # 原为固定20
            y = self.grid_max_radius * math.sin(rad)
            grid_path.moveTo(0, 0)
            grid_path.lineTo(x, y)
        
        painter.drawPath(grid_path)

    def _draw_crosshair(self, painter):
        """十字准星绘制"""
        cross_pen = QtGui.QPen(self.crosshair_color, 0)
        cross_pen.setDashPattern([2, 2])
        painter.setPen(cross_pen)
        
        # 使用浮点数坐标确保绘制精度
        painter.drawLine(QtCore.QLineF(-20.0, 0.0, 20.0, 0.0))
        painter.drawLine(QtCore.QLineF(0.0, -20.0, 0.0, 20.0))

    def _draw_selection_rect(self, painter):
        """选择框绘制"""
        if self.selection_rect:
            painter.setPen(QtGui.QPen(QtCore.Qt.yellow, 0))
            painter.drawRect(self.selection_rect)


    
    def mousePressEvent(self, event):
        """画布鼠标按下事件"""
        if event.button() == QtCore.Qt.LeftButton:
            self.dragging = True
            self.last_mouse_pos = event.pos()
            self.selection_start = self._map_to_scene(event.pos())
        elif event.button() == QtCore.Qt.RightButton:
            self.dragging = True
            self.last_mouse_pos = event.pos()
        else:  # 处理其他按钮事件
            super().mousePressEvent(event)

    def mouseDoubleClickEvent(self, event):
        """双击左键重置视图"""
        if event.button() == QtCore.Qt.LeftButton:
            self.view_offset = QtCore.QPoint(0, 0)
            self.current_scale = DEFAULT_SCALE
            self.update()

    def mouseMoveEvent(self, event):
        """画布鼠标移动事件"""
        if not self.dragging or not self.last_mouse_pos:
            return  # 避免未初始化导致异常
        if self.dragging:
            if event.buttons() & QtCore.Qt.LeftButton:
                # 视图平移
                delta = event.pos() - self.last_mouse_pos
                self.view_offset += delta
                self.last_mouse_pos = event.pos()
                self.update()
            elif event.buttons() & QtCore.Qt.RightButton:
                # 框选操作
                end_pos = self._map_to_scene(event.pos())
                self.selection_rect = QtCore.QRectF(
                    self.selection_start,
                    end_pos
                ).normalized()
                self.update()

    def mouseReleaseEvent(self, event):
        """画布鼠标释放事件"""
        self.dragging = False
        if self.selection_rect:
            self._process_selection()
            self.selection_rect = None
            self.update()

    def wheelEvent(self, event):
        """画布滚轮缩放事件"""
        zoom_factor = 1.2 if event.angleDelta().y() > 0 else 0.8
        mouse_before = self._map_to_scene(event.pos())
        
        self.current_scale = max(0.1, min(self.current_scale * zoom_factor, 100.0))
        
        mouse_after = self._map_to_scene(event.pos())
        delta = mouse_after - mouse_before
        self.view_offset += (delta * self.current_scale).toPoint()
        self.update()

    def _map_to_scene(self, pos):
        """精确坐标转换方法"""
        view_center = self.rect().center() + self.view_offset
        return QtCore.QPointF(
            (pos.x() - view_center.x()) / self.current_scale,
            (view_center.y() - pos.y()) / self.current_scale  # Y轴翻转
        )

    def _process_selection(self):
        """优化的框选处理"""
        if not self.selection_rect:
            return
            
        selected = [
            (x, y) for x, y in self.points 
            if self.selection_rect.contains(QtCore.QPointF(x, y))
        ]
        print(f"选中点数: {len(selected)} 范围: {self.selection_rect.getCoords()}")


class DisplayUI(BaseSubLayout):
    def __init__(self, DroneFunc:Func):
        super().__init__(QtCore.Qt.Vertical, show_border=False)
        self.DroneFunc = DroneFunc
        
        # 初始化自定义画布
        self.canvas = DisplayCanvas()
        self.add_widgets([self.canvas])
        
        # 初始化交互功能
        # self._init_context_menu()
        self._setup_timer()
        
        # 视图参数
        self.dragging = False
        self.last_mouse_pos = QtCore.QPoint()


    def _setup_timer(self):
        """定时器配置"""
        self.timer = QtCore.QTimer()
        self.timer.setInterval(100)  # 100ms刷新间隔
        self.timer.timeout.connect(self._update_display)
        self.start_timer_signal.emit()

    def _update_display(self):
        """数据更新方法"""
        cur_x, cur_y, cur_z, cur_roll, cur_pitch, cur_yaw = self.DroneFunc.get_drone_pose()
        # self.DroneFunc._drone_node.get_logger().info(f"cur_x = {cur_x}, cur_y = {cur_y}")
        points = []
        points.append((cur_x, cur_y))
        self.canvas.points = points
        self.canvas.update()

    # ---------- 事件处理增强 ----------
    def mousePressEvent(self, event):
        """鼠标按下事件路由"""
        self.canvas.mousePressEvent(event)

    def mouseMoveEvent(self, event):
        """鼠标移动事件路由"""
        self.canvas.mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        """鼠标释放事件路由"""
        self.canvas.mouseReleaseEvent(event)

    def wheelEvent(self, event):
        """滚轮事件路由"""
        self.canvas.wheelEvent(event)
