"""
地图控件坐标转换与渲染单元测试

不依赖 ROS 环境，仅依赖 PyQt5 + numpy。
"""
import pytest

pytest.importorskip("PyQt5")
pytest.importorskip("numpy")

import numpy as np

from robot_gui.map_widget import MapWidget


def _make_widget(qapp):
    """构造一个地图元数据已配置好的 MapWidget（不 show）"""
    w = MapWidget()
    w.resize(400, 400)
    # 地图：原点 (-5,-5)，200x200 格，分辨率 0.05 -> 覆盖世界坐标 [-5, 5] x [-5, 5]
    w._map_resolution = 0.05
    w._map_origin_x = -5.0
    w._map_origin_y = -5.0
    w._map_width = 200
    w._map_height = 200
    w._zoom = 1.0
    w._offset_x = 0.0
    w._offset_y = 0.0
    w._has_map = True
    return w


def test_world_pixel_roundtrip(qapp):
    """世界坐标 -> 像素 -> 世界坐标 应可逆"""
    w = _make_widget(qapp)
    cases = [(0.0, 0.0), (1.23, -2.34), (-4.5, 4.5), (3.14, 0.0), (5.0, -5.0)]
    for wx, wy in cases:
        p = w._world_to_pixel(wx, wy)
        rx, ry = w._pixel_to_world(p.x(), p.y())
        assert abs(rx - wx) < 1e-6, f"x roundtrip failed: {wx} -> {rx}"
        assert abs(ry - wy) < 1e-6, f"y roundtrip failed: {wy} -> {ry}"


def test_world_origin_at_pixel_center(qapp):
    """世界坐标 (0,0) 应映射到控件中心（地图中心）"""
    w = _make_widget(qapp)
    p = w._world_to_pixel(0.0, 0.0)
    assert abs(p.x() - 200.0) < 1e-6
    assert abs(p.y() - 200.0) < 1e-6


def test_zoom_changes_pixel_distance(qapp):
    """放大缩放后，相同世界距离对应的像素距离应按比例变化"""
    w = _make_widget(qapp)
    p1 = w._world_to_pixel(0.0, 0.0)
    p2 = w._world_to_pixel(1.0, 0.0)
    base = p2.x() - p1.x()

    w._zoom = 2.0
    p1b = w._world_to_pixel(0.0, 0.0)
    p2b = w._world_to_pixel(1.0, 0.0)
    doubled = p2b.x() - p1b.x()
    assert abs(doubled - 2.0 * base) < 1e-6


def test_rebuild_map_pixmap_dimensions(qapp):
    """向量化重建后的 pixmap 尺寸应与数据一致"""
    w = _make_widget(qapp)
    data = np.array([[-1, 0, 50, 100]], dtype=np.int8)  # 1 行 4 列
    w._map_data = data
    w._map_dirty = True
    w._rebuild_map_pixmap()
    assert w._map_pixmap is not None
    assert not w._map_pixmap.isNull()
    assert w._map_pixmap.width() == 4
    assert w._map_pixmap.height() == 1


def test_rebuild_map_pixmap_handles_all_value_classes(qapp):
    """未知/空闲/占用三类值都应能正常生成 pixmap（不报错即通过）"""
    w = _make_widget(qapp)
    data = np.array([
        [-1, -1,  0,  0],
        [ 0,  1, 50, 100],
        [-1,  0, 25, 75],
    ], dtype=np.int8)
    w._map_data = data
    w._map_dirty = True
    w._rebuild_map_pixmap()
    assert w._map_pixmap is not None
    assert w._map_pixmap.width() == 4
    assert w._map_pixmap.height() == 3
