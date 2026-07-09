"""
Gazebo 仿真视图组件

把 Gazebo 里一个固定角度相机的图片流嵌进 GUI。
- image_msg_to_qimage: ROS sensor_msgs/Image -> QImage 的纯函数（可单测）
- GazeboViewWidget: 显示最新一帧，自动按比例缩放
"""

from typing import Optional

from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap


def image_msg_to_qimage(msg) -> QImage:
    """
    将 ROS sensor_msgs/Image 转成自包含的 QImage（已 .copy()，安全跨线程传递）

    支持 rgb8 / bgr8 / rgba8 / bgra8 / mono8。
    其它编码返回空 QImage。
    """
    w = int(getattr(msg, 'width', 0))
    h = int(getattr(msg, 'height', 0))
    step = int(getattr(msg, 'step', 0))
    if w <= 0 or h <= 0 or step <= 0:
        return QImage()
    enc = str(getattr(msg, 'encoding', '')).lower()
    data = bytes(msg.data)
    if enc == 'rgb8':
        return QImage(data, w, h, step, QImage.Format_RGB888).copy()
    if enc == 'bgr8':
        return QImage(data, w, h, step, QImage.Format_BGR888).copy()
    if enc == 'rgba8':
        return QImage(data, w, h, step, QImage.Format_RGBA8888).copy()
    if enc == 'bgra8':
        return QImage(data, w, h, step, QImage.Format_BGRA8888).copy()
    if enc == 'mono8':
        return QImage(data, w, h, step, QImage.Format_Grayscale8).copy()
    return QImage()


class GazeboViewWidget(QWidget):
    """显示 Gazebo 相机图片流的控件"""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._latest_pixmap: Optional[QPixmap] = None

        self._label = QLabel('等待 Gazebo 相机数据...\n\n请先启动仿真（会自动注入相机）')
        self._label.setAlignment(Qt.AlignCenter)
        self._label.setStyleSheet('color:#8b7355; font-size:15px; background-color:#f5f0e1;')
        self._label.setMinimumSize(320, 240)

        ly = QVBoxLayout(self)
        ly.setContentsMargins(0, 0, 0, 0)
        ly.addWidget(self._label)
        self.setMinimumSize(320, 240)

    @pyqtSlot(QImage)
    def update_image(self, img: QImage):
        if img is None or img.isNull():
            return
        pm = QPixmap.fromImage(img)
        self._latest_pixmap = pm
        self._label.setText('')
        self._label.setPixmap(pm.scaled(
            self._label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
        ))

    def clear_view(self):
        self._latest_pixmap = None
        self._label.setPixmap(QPixmap())
        self._label.setText('等待 Gazebo 相机数据...\n\n请先启动仿真（会自动注入相机）')

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self._latest_pixmap is not None:
            self._label.setPixmap(self._latest_pixmap.scaled(
                self._label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
            ))
