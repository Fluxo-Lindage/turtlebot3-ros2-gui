"""
实时地图可视化组件

提供类似 RViz2 的地图显示功能：
  - 实时渲染 OccupancyGrid 地图（SLAM 建图 / 导航地图）
  - 显示机器人当前位置和朝向
  - 支持鼠标滚轮缩放
  - 支持鼠标拖动平移
  - 点击地图设置导航目标点（类似 RViz2 的 "2D Goal Pose"）

颜色映射:
  - 未知区域 (-1): 浅灰色
  - 空闲区域 (0): 白色
  - 障碍物 (1-100): 深灰色 → 黑色
  - 机器人: 蓝色圆点 + 方向箭头
  - 导航目标: 红色旗帜标记
"""

import math
import numpy as np
from typing import Optional, Tuple

from PyQt5.QtWidgets import QWidget, QRubberBand
from PyQt5.QtCore import Qt, QPoint, QPointF, QRect, pyqtSignal
from PyQt5.QtGui import (
    QPainter, QPixmap, QImage, QColor, QPen, QBrush,
    QFont, QFontMetrics, QPolygonF, QWheelEvent, QMouseEvent,
    QPaintEvent, QResizeEvent,
)


# ===================== 颜色常量 =====================
COLOR_UNKNOWN = QColor(210, 200, 180)       # 未知区域 — 暖灰
COLOR_FREE = QColor(252, 247, 235)          # 空闲区域 — 米白
COLOR_OCCUPIED_LOW = QColor(190, 180, 160)  # 低占用
COLOR_OCCUPIED_HIGH = QColor(60, 50, 35)    # 高占用 — 深棕
COLOR_ROBOT = QColor(90, 130, 70)           # 机器人 — 橄榄绿
COLOR_ROBOT_ARROW = QColor(255, 255, 255)   # 方向箭头 — 白色
COLOR_GOAL = QColor(200, 75, 60)            # 目标点 — 暗红
COLOR_GOAL_ARROW = QColor(255, 255, 255)
COLOR_BACKGROUND = QColor(240, 235, 218)    # 地图外背景 — 米色
COLOR_GRID_LINE = QColor(220, 212, 195)     # 网格线
COLOR_ROBOT_TRAIL = QColor(90, 130, 70, 80)  # 轨迹 — 半透明橄榄绿


class MapWidget(QWidget):
    """
    实时地图显示控件

    信号:
        goal_selected(x, y, yaw): 用户在点击地图设置了导航目标
    """

    goal_selected = pyqtSignal(float, float, float)  # x, y, yaw (世界坐标系)

    # 最小/最大缩放级别
    MIN_ZOOM = 0.2
    MAX_ZOOM = 10.0

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)

        # ---- 地图数据 ----
        self._map_data: Optional[np.ndarray] = None   # 占据栅格数据 (h, w)
        self._map_width: int = 0                       # 栅格宽度 (cells)
        self._map_height: int = 0                      # 栅格高度 (cells)
        self._map_resolution: float = 0.05             # 分辨率 (m/cell)
        self._map_origin_x: float = 0.0                # 原点 X (世界坐标)
        self._map_origin_y: float = 0.0                # 原点 Y (世界坐标)
        self._has_map: bool = False                    # 是否收到过地图

        # ---- 地图缓存（避免每帧重绘） ----
        self._map_pixmap: Optional[QPixmap] = None
        self._map_dirty: bool = True

        # ---- 机器人状态 ----
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_yaw: float = 0.0
        self._robot_odom_ok: bool = False
        self._robot_trail: list = []  # [(x, y), ...] 轨迹点

        # ---- 导航目标 ----
        self._goal_x: Optional[float] = None
        self._goal_y: Optional[float] = None
        self._goal_yaw: float = 0.0
        self._has_goal: bool = False

        # ---- 视角变换 ----
        self._zoom: float = 1.0
        self._offset_x: float = 0.0    # 平移偏移 (像素)
        self._offset_y: float = 0.0

        # ---- 交互状态 ----
        self._panning: bool = False
        self._last_mouse_pos: Optional[QPoint] = None
        self._dragging_goal: bool = False  # 拖拽目标方向
        self._goal_drag_start: Optional[QPointF] = None

        # ---- 外观 ----
        self._robot_size: int = 16       # 机器人图标大小 (像素)
        self._goal_size: int = 14        # 目标点图标大小 (像素)
        self._show_grid: bool = True     # 是否显示网格

        # 设置控件属性
        self.setMinimumSize(300, 250)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setCursor(Qt.OpenHandCursor)

        # 背景色
        self.setAutoFillBackground(True)
        pal = self.palette()
        pal.setColor(self.backgroundRole(), COLOR_BACKGROUND)
        self.setPalette(pal)

    # ===================== 公共接口 =====================

    def update_map(self,
                   data: np.ndarray,
                   resolution: float,
                   origin_x: float,
                   origin_y: float,
                   width: int,
                   height: int):
        """
        更新地图数据（由 ROS 线程调用，线程安全）

        Args:
            data: 占据栅格数组 (height, width)，值范围 [-1, 100]
            resolution: 分辨率 (m/cell)
            origin_x, origin_y: 地图原点在世界坐标系中的位置
            width, height: 栅格尺寸
        """
        self._map_data = data.copy()
        self._map_resolution = resolution
        self._map_origin_x = origin_x
        self._map_origin_y = origin_y
        self._map_width = width
        self._map_height = height
        was_first_map = not self._has_map
        self._has_map = True
        self._map_dirty = True

        # 首次收到地图时，自动适应窗口
        if was_first_map:
            self._fit_map_to_view()

        self.update()  # 触发重绘

    def update_robot_pose(self, x: float, y: float, yaw: float):
        """
        更新机器人位姿

        Args:
            x, y: 世界坐标 (m)
            yaw: 偏航角 (rad)
        """
        self._robot_x = x
        self._robot_y = y
        self._robot_yaw = yaw
        self._robot_odom_ok = True

        # 更新轨迹（世界坐标）
        self._robot_trail.append((x, y))
        if len(self._robot_trail) > 500:  # 限制轨迹点数
            self._robot_trail = self._robot_trail[-500:]

        self.update()

    def set_goal(self, x: float, y: float, yaw: float = 0.0):
        """设置导航目标点"""
        self._goal_x = x
        self._goal_y = y
        self._goal_yaw = yaw
        self._has_goal = True
        self.update()

    def clear_goal(self):
        """清除导航目标"""
        self._has_goal = False
        self._goal_x = None
        self._goal_y = None
        self.update()

    def clear_map(self):
        """完全清除地图和所有标记（切换仿真环境时调用）"""
        self._map_data = None
        self._map_pixmap = None
        self._map_dirty = True
        self._has_map = False
        self._has_goal = False
        self._goal_x = None
        self._goal_y = None
        self._robot_trail.clear()
        # 重置机器人位姿缓存，避免清除后还按上一张地图的旧坐标画小车
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._robot_odom_ok = False
        self.update()

    def clear_robot_trail(self):
        """清除机器人轨迹"""
        self._robot_trail.clear()
        self.update()

    def fit_map_to_view(self):
        """缩放地图使其适合当前控件大小"""
        self._fit_map_to_view()
        self.update()

    def reset_view(self):
        """重置视角（居中、缩放=1）"""
        self._zoom = 1.0
        self._offset_x = 0.0
        self._offset_y = 0.0
        self.update()

    # ===================== 坐标转换 =====================

    def _world_to_pixel(self, wx: float, wy: float) -> QPointF:
        """
        世界坐标 → 屏幕像素坐标

        Args:
            wx, wy: 世界坐标 (m)

        Returns:
            QPointF: 屏幕像素位置
        """
        # 世界坐标 → 图像坐标
        ix = (wx - self._map_origin_x) / self._map_resolution
        iy = (wy - self._map_origin_y) / self._map_resolution

        # 图像 Y 轴翻转（Qt 坐标系 Y 向下）
        iy = self._map_height - iy

        # 图像坐标 → 屏幕坐标（应用缩放和平移）
        cx = self.width() / 2.0
        cy = self.height() / 2.0
        sx = cx + (ix - self._map_width / 2.0) * self._zoom + self._offset_x
        sy = cy + (iy - self._map_height / 2.0) * self._zoom + self._offset_y

        return QPointF(sx, sy)

    def _pixel_to_world(self, px: float, py: float) -> Tuple[float, float]:
        """
        屏幕像素坐标 → 世界坐标

        Args:
            px, py: 屏幕像素位置

        Returns:
            (wx, wy): 世界坐标 (m)
        """
        # 屏幕坐标 → 图像坐标
        cx = self.width() / 2.0
        cy = self.height() / 2.0
        ix = (px - cx - self._offset_x) / self._zoom + self._map_width / 2.0
        iy = (py - cy - self._offset_y) / self._zoom + self._map_height / 2.0

        # 图像 Y 轴翻转
        iy = self._map_height - iy

        # 图像坐标 → 世界坐标
        wx = ix * self._map_resolution + self._map_origin_x
        wy = iy * self._map_resolution + self._map_origin_y

        return wx, wy

    # ===================== 绘制 =====================

    def paintEvent(self, event: QPaintEvent):
        """绘制事件"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setRenderHint(QPainter.SmoothPixmapTransform, False)

        # 背景
        painter.fillRect(self.rect(), COLOR_BACKGROUND)

        # 绘制地图
        if self._has_map and self._map_data is not None:
            self._paint_map(painter)
            self._paint_grid(painter)
            self._paint_robot_trail(painter)

        # 绘制导航目标
        if self._has_goal and self._goal_x is not None:
            self._paint_goal(painter)

        # 绘制机器人（画在最上层）
        if self._robot_odom_ok:
            self._paint_robot(painter)

        # 无地图时的提示
        if not self._has_map:
            self._paint_no_map_hint(painter)

        # 边框
        painter.setPen(QPen(QColor(180, 170, 150), 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawRect(self.rect().adjusted(1, 1, -1, -1))

    def _paint_map(self, painter: QPainter):
        """绘制占据栅格地图"""
        if self._map_dirty:
            self._rebuild_map_pixmap()
            self._map_dirty = False

        if self._map_pixmap is None:
            return

        # 计算地图在屏幕上的绘制区域
        cx = self.width() / 2.0
        cy = self.height() / 2.0

        # 地图图像尺寸（像素）
        img_w = self._map_width * self._zoom
        img_h = self._map_height * self._zoom

        # 地图左上角在屏幕上的位置
        top_left_x = cx - (self._map_width / 2.0) * self._zoom + self._offset_x
        top_left_y = cy - (self._map_height / 2.0) * self._zoom + self._offset_y

        target_rect = QRect(
            int(top_left_x), int(top_left_y),
            int(img_w), int(img_h)
        )

        painter.drawPixmap(target_rect, self._map_pixmap)

    def _rebuild_map_pixmap(self):
        """重建地图 Pixmap 缓存（numpy 向量化，避免逐像素 Python 循环）"""
        if self._map_data is None:
            return

        h, w = self._map_data.shape
        if h == 0 or w == 0:
            return

        data = self._map_data

        # 构建 RGB 数组，默认填“未知区域”颜色
        rgb = np.empty((h, w, 3), dtype=np.uint8)
        rgb[:, :, 0] = COLOR_UNKNOWN.red()
        rgb[:, :, 1] = COLOR_UNKNOWN.green()
        rgb[:, :, 2] = COLOR_UNKNOWN.blue()

        # 空闲区域
        free_mask = (data == 0)
        rgb[free_mask, 0] = COLOR_FREE.red()
        rgb[free_mask, 1] = COLOR_FREE.green()
        rgb[free_mask, 2] = COLOR_FREE.blue()

        # 占用区域 (1-100)：在 LOW 与 HIGH 之间按 val/100 线性插值
        occ_mask = (data > 0)
        if np.any(occ_mask):
            t = data[occ_mask].astype(np.float32) / 100.0  # (N,)
            low = np.array([COLOR_OCCUPIED_LOW.red(), COLOR_OCCUPIED_LOW.green(), COLOR_OCCUPIED_LOW.blue()], dtype=np.float32)
            high = np.array([COLOR_OCCUPIED_HIGH.red(), COLOR_OCCUPIED_HIGH.green(), COLOR_OCCUPIED_HIGH.blue()], dtype=np.float32)
            # t[:, None] (N,1) 与 low/high (3,) 广播 -> (N,3)
            rgb[occ_mask] = (low + (high - low) * t[:, None]).astype(np.uint8)

        # numpy buffer -> QImage。用 .copy() 防止数组被回收后 QImage 引用悬空
        # bytesPerLine = w * 3（每像素 3 字节，RGB 顺序）
        image = QImage(rgb.data, w, h, int(w * 3), QImage.Format_RGB888).copy()
        self._map_pixmap = QPixmap.fromImage(image)

    def _paint_grid(self, painter: QPainter):
        """绘制坐标网格线（仅在缩放足够大时显示）"""
        if not self._show_grid or self._zoom < 0.8:
            return

        pen = QPen(COLOR_GRID_LINE, 1, Qt.DotLine)
        painter.setPen(pen)

        # 计算可见的世界范围
        cx = self.width() / 2.0
        cy = self.height() / 2.0

        # 1 米在屏幕上大约多少像素
        meter_pixels = self._zoom / self._map_resolution

        # 根据缩放级别决定网格间距
        if meter_pixels > 80:
            spacing_m = 0.5
        elif meter_pixels > 40:
            spacing_m = 1.0
        else:
            spacing_m = 2.0

        # 以地图原点为基准画网格
        origin_px = self._world_to_pixel(0.0, 0.0)
        origin_px_x = origin_px.x()
        origin_px_y = origin_px.y()

        spacing_px = spacing_m * meter_pixels

        # 垂直线
        if spacing_px > 15:
            # 向右
            x = origin_px_x
            while x < self.width():
                painter.drawLine(int(x), 0, int(x), self.height())
                x += spacing_px
            # 向左
            x = origin_px_x - spacing_px
            while x > 0:
                painter.drawLine(int(x), 0, int(x), self.height())
                x -= spacing_px

            # 水平线
            y = origin_px_y
            while y < self.height():
                painter.drawLine(0, int(y), self.width(), int(y))
                y += spacing_px
            y = origin_px_y - spacing_px
            while y > 0:
                painter.drawLine(0, int(y), self.width(), int(y))
                y -= spacing_px

    def _paint_robot(self, painter: QPainter):
        """绘制机器人位置和朝向"""
        pos = self._world_to_pixel(self._robot_x, self._robot_y)
        px, py = pos.x(), pos.y()

        # 检查是否在可见区域内
        if px < -50 or px > self.width() + 50 or py < -50 or py > self.height() + 50:
            return

        # 机器人大小随缩放变化
        size = max(10, min(40, self._robot_size * self._zoom))

        # — 绘制机器人身体（圆形） —
        painter.setPen(QPen(COLOR_ROBOT.darker(120), 2))
        painter.setBrush(QBrush(COLOR_ROBOT))
        painter.drawEllipse(QPointF(px, py), size / 2, size / 2)

        # — 绘制方向箭头 —
        arrow_len = size * 0.8
        arrow_tip = QPointF(
            px + arrow_len * math.cos(self._robot_yaw),
            py - arrow_len * math.sin(self._robot_yaw)  # Qt Y 轴翻转
        )

        # 箭头两侧
        arrow_angle = math.pi / 6  # 30度
        left_wing = QPointF(
            arrow_tip.x() - (size * 0.4) * math.cos(self._robot_yaw - arrow_angle),
            arrow_tip.y() + (size * 0.4) * math.sin(self._robot_yaw - arrow_angle)
        )
        right_wing = QPointF(
            arrow_tip.x() - (size * 0.4) * math.cos(self._robot_yaw + arrow_angle),
            arrow_tip.y() + (size * 0.4) * math.sin(self._robot_yaw + arrow_angle)
        )

        arrow = QPolygonF([arrow_tip, left_wing, right_wing])
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(COLOR_ROBOT_ARROW))
        painter.drawPolygon(arrow)

    def _paint_robot_trail(self, painter: QPainter):
        """绘制机器人运动轨迹"""
        if len(self._robot_trail) < 2:
            return

        painter.setPen(QPen(COLOR_ROBOT_TRAIL, 2))
        painter.setBrush(Qt.NoBrush)

        points = []
        for wx, wy in self._robot_trail[-200:]:  # 只画最近200个点
            pt = self._world_to_pixel(wx, wy)
            points.append(pt)

        for i in range(1, len(points)):
            painter.drawLine(points[i - 1], points[i])

    def _paint_goal(self, painter: QPainter):
        """绘制导航目标点标记"""
        if self._goal_x is None or self._goal_y is None:
            return

        pos = self._world_to_pixel(self._goal_x, self._goal_y)
        px, py = pos.x(), pos.y()

        size = max(8, min(30, self._goal_size * self._zoom))

        # — 红色旗帜标记 —
        painter.setPen(QPen(COLOR_GOAL.darker(120), 2))
        painter.setBrush(QBrush(COLOR_GOAL))

        # 旗杆
        pole_top = QPointF(px, py - size)
        pole_bottom = QPointF(px, py + size * 0.5)
        painter.setPen(QPen(QColor(100, 100, 100), max(2, int(size / 6))))
        painter.drawLine(pole_top, pole_bottom)

        # 旗帜（三角形）
        flag_points = QPolygonF([
            QPointF(px, py - size),
            QPointF(px + size * 0.7, py - size * 0.7),
            QPointF(px, py - size * 0.4),
        ])
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(COLOR_GOAL))
        painter.drawPolygon(flag_points)

        # — 目标朝向箭头 —
        if self._goal_yaw != 0.0:
            arrow_len = size * 1.2
            arrow_tip = QPointF(
                px + arrow_len * math.cos(self._goal_yaw),
                py - arrow_len * math.sin(self._goal_yaw)
            )
            painter.setPen(QPen(COLOR_GOAL, 2))
            painter.drawLine(QPointF(px, py), arrow_tip)

        # 坐标文字
        font = QFont('Sans', max(9, int(10 * self._zoom)))
        painter.setFont(font)
        painter.setPen(QColor(60, 60, 60))
        text = f'({self._goal_x:.2f}, {self._goal_y:.2f})'
        text_rect = painter.boundingRect(
            QRect(int(px + size), int(py - size * 1.5), 200, 30),
            Qt.AlignLeft, text
        )
        # 白色背景
        painter.fillRect(text_rect.adjusted(-2, -1, 2, 1), QColor(255, 255, 255, 200))
        painter.drawText(text_rect, Qt.AlignLeft, text)

    def _paint_no_map_hint(self, painter: QPainter):
        """绘制无地图时的提示信息"""
        painter.setPen(QColor(150, 140, 120))
        font = QFont('Sans', 16)
        painter.setFont(font)
        painter.drawText(
            self.rect(),
            Qt.AlignCenter,
            '等待地图数据...\n\n'
            '启动 SLAM 建图或加载导航地图后\n'
            '此处将实时显示地图\n\n'
            '点击地图设置导航目标\n'
            '滚轮缩放  |  拖动平移'
        )

    # ===================== 事件处理 =====================

    def wheelEvent(self, event: QWheelEvent):
        """鼠标滚轮缩放"""
        zoom_factor = 1.1
        old_zoom = self._zoom

        if event.angleDelta().y() > 0:
            new_zoom = self._zoom * zoom_factor
        else:
            new_zoom = self._zoom / zoom_factor

        # 限制缩放范围
        new_zoom = max(self.MIN_ZOOM, min(self.MAX_ZOOM, new_zoom))

        if new_zoom != old_zoom:
            # 以鼠标位置为中心缩放
            mouse_x = event.position().x()
            mouse_y = event.position().y()
            cx = self.width() / 2.0
            cy = self.height() / 2.0

            # 调整偏移使鼠标指向的内容保持不变
            ratio = new_zoom / old_zoom
            self._offset_x = mouse_x - ratio * (mouse_x - cx - self._offset_x) - cx
            self._offset_y = mouse_y - ratio * (mouse_y - cy - self._offset_y) - cy
            self._zoom = new_zoom
            self.update()

    def mousePressEvent(self, event: QMouseEvent):
        """鼠标按下事件"""
        if not self._has_map:
            return

        if event.button() == Qt.MiddleButton or (
            event.button() == Qt.LeftButton and event.modifiers() & Qt.ControlModifier
        ):
            # 中键 或 Ctrl+左键 → 平移
            self._panning = True
            self._last_mouse_pos = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
        elif event.button() == Qt.LeftButton:
            # 左键 → 设置导航目标
            self._set_goal_from_click(event.pos())
        elif event.button() == Qt.RightButton:
            # 右键 → 清除目标
            self.clear_goal()

    def mouseMoveEvent(self, event: QMouseEvent):
        """鼠标移动事件"""
        if self._panning and self._last_mouse_pos is not None:
            delta = event.pos() - self._last_mouse_pos
            self._offset_x += delta.x()
            self._offset_y += delta.y()
            self._last_mouse_pos = event.pos()
            self.update()

        # 更新鼠标位置提示
        if self._has_map:
            wx, wy = self._pixel_to_world(event.pos().x(), event.pos().y())
            self.setToolTip(f'世界坐标: ({wx:.3f}, {wy:.3f}) m  |  '
                           f'像素: ({event.pos().x()}, {event.pos().y()})  |  '
                           f'缩放: {self._zoom:.1f}x')

    def mouseReleaseEvent(self, event: QMouseEvent):
        """鼠标释放事件"""
        if event.button() == Qt.MiddleButton or (
            event.button() == Qt.LeftButton and self._panning
        ):
            self._panning = False
            self._last_mouse_pos = None
            self.setCursor(Qt.OpenHandCursor)

    def mouseDoubleClickEvent(self, event: QMouseEvent):
        """双击重置视角"""
        if event.button() == Qt.LeftButton:
            self._fit_map_to_view()
            self.update()

    def resizeEvent(self, event: QResizeEvent):
        """窗口大小变化事件"""
        super().resizeEvent(event)

        # 如果还没有地图数据，首次收到时会自动 fit
        # 否则保持当前视角
        self.update()

    def keyPressEvent(self, event):
        """键盘事件"""
        if event.key() == Qt.Key_R:
            # R 键重置视角
            self.reset_view()
        elif event.key() == Qt.Key_F:
            # F 键适应窗口
            self._fit_map_to_view()
            self.update()
        elif event.key() == Qt.Key_G:
            # G 键切换网格
            self._show_grid = not self._show_grid
            self.update()
        elif event.key() == Qt.Key_C:
            # C 键清除轨迹
            self.clear_robot_trail()
        elif event.key() == Qt.Key_Escape:
            # Esc 清除目标
            self.clear_goal()
        else:
            super().keyPressEvent(event)

    # ===================== 内部方法 =====================

    def _set_goal_from_click(self, click_pos: QPoint):
        """从鼠标点击位置设置导航目标"""
        wx, wy = self._pixel_to_world(click_pos.x(), click_pos.y())

        # 计算目标朝向（默认指向远离当前位置的方向，或朝北）
        if self._robot_odom_ok:
            yaw = math.atan2(wy - self._robot_y, wx - self._robot_x)
        else:
            yaw = 0.0

        self._goal_x = wx
        self._goal_y = wy
        self._goal_yaw = yaw
        self._has_goal = True
        self.update()

        self.goal_selected.emit(wx, wy, yaw)

    def _fit_map_to_view(self):
        """自动调整缩放使地图适应窗口"""
        if not self._has_map or self._map_width == 0 or self._map_height == 0:
            return

        widget_w = self.width()
        widget_h = self.height()

        if widget_w <= 0 or widget_h <= 0:
            return

        # 留 10% 边距
        margin = 0.9
        zoom_w = widget_w / self._map_width * margin
        zoom_h = widget_h / self._map_height * margin

        self._zoom = min(zoom_w, zoom_h)
        self._offset_x = 0.0
        self._offset_y = 0.0
