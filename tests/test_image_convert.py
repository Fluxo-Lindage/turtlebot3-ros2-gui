"""
image_msg_to_qimage 转换单元测试

不依赖 ROS，用 types.SimpleNamespace 构造假 Image 消息。
"""
import types

import pytest

pytest.importorskip("PyQt5")

from PyQt5.QtGui import QImage
from robot_gui.gazebo_view import image_msg_to_qimage


def _fake_image(width, height, encoding, pixels):
    """pixels: 扁平的字节列表，长度应为 width*height*channels"""
    data = bytes(pixels)
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}[encoding]
    step = width * channels
    return types.SimpleNamespace(
        width=width, height=height, step=step, encoding=encoding, data=data
    )


def test_rgb8_dimensions_and_pixels(qapp):
    msg = _fake_image(2, 2, 'rgb8', [
        255, 0, 0,    0, 255, 0,
        0, 0, 255,    255, 255, 255,
    ])
    img = image_msg_to_qimage(msg)
    assert not img.isNull()
    assert img.width() == 2
    assert img.height() == 2
    assert img.pixelColor(0, 0).red() == 255
    assert img.pixelColor(0, 0).green() == 0
    assert img.pixelColor(1, 0).green() == 255
    assert img.pixelColor(0, 1).blue() == 255
    assert img.pixelColor(1, 1).red() == 255
    assert img.pixelColor(1, 1).green() == 255


def test_bgr8_pixel_order(qapp):
    # bgr8: 数据顺序是 B,G,R。像素 (255,0,0) 在 bgr 里表示 B=255 -> 蓝色
    msg = _fake_image(1, 1, 'bgr8', [255, 0, 0])
    img = image_msg_to_qimage(msg)
    assert not img.isNull()
    c = img.pixelColor(0, 0)
    assert c.blue() == 255
    assert c.red() == 0


def test_mono8(qapp):
    msg = _fake_image(2, 1, 'mono8', [0, 255])
    img = image_msg_to_qimage(msg)
    assert not img.isNull()
    assert img.width() == 2
    assert img.height() == 1


def test_unsupported_encoding_returns_null(qapp):
    msg = _fake_image(1, 1, 'rgb8', [0, 0, 0])
    msg.encoding = 'yuv422'
    img = image_msg_to_qimage(msg)
    assert img.isNull()


def test_invalid_dimensions_returns_null(qapp):
    msg = types.SimpleNamespace(width=0, height=0, step=0, encoding='rgb8', data=b'')
    img = image_msg_to_qimage(msg)
    assert img.isNull()


def test_converted_image_is_self_contained(qapp):
    """转换后 QImage 应自包含（.copy()），原 data 释放后仍可用"""
    msg = _fake_image(1, 1, 'rgb8', [10, 20, 30])
    img = image_msg_to_qimage(msg)
    del msg  # 释放原数据
    assert not img.isNull()
    assert img.pixelColor(0, 0).red() == 10
