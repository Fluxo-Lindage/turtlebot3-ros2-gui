"""
四元数 <-> 偏航角 转换单元测试

依赖 ROS2 环境（rclpy / geometry_msgs），在未安装时自动 skip。
运行方式：source /opt/ros/humble/setup.bash && python -m pytest tests/test_quaternion.py -v
"""
import math

import pytest

rclpy = pytest.importorskip("rclpy")  # 未装 ROS2 则跳过整组测试

from robot_gui.ros_node import RosRobotNode


def _angle_diff(a: float, b: float) -> float:
    """归一化到 [-pi, pi] 的角度差绝对值"""
    d = (a - b + math.pi) % (2 * math.pi) - math.pi
    return abs(d)


def test_yaw_to_quaternion_components():
    """yaw=0 时四元数应为 (0,0,0,1)；yaw=pi 时 z≈1, w≈0"""
    q0 = RosRobotNode._yaw_to_quaternion(0.0)
    assert q0.x == 0.0 and q0.y == 0.0
    assert abs(q0.z - 0.0) < 1e-9
    assert abs(q0.w - 1.0) < 1e-9

    qpi = RosRobotNode._yaw_to_quaternion(math.pi)
    assert abs(qpi.z - 1.0) < 1e-9
    assert abs(qpi.w) < 1e-9


def test_quaternion_yaw_roundtrip():
    """yaw -> quaternion -> yaw 应可逆（处理 ±pi 包绕）"""
    yaws = [0.0, math.pi / 4, math.pi / 2, math.pi,
            -math.pi / 2, -math.pi / 4, 1.0, -2.0, 2.5]
    for yaw in yaws:
        q = RosRobotNode._yaw_to_quaternion(yaw)
        yaw_back = RosRobotNode._quaternion_to_yaw(q.x, q.y, q.z, q.w)
        assert _angle_diff(yaw_back, yaw) < 1e-9, (
            f"roundtrip 失败: yaw={yaw} -> back={yaw_back}"
        )


def test_quaternion_yaw_known_values():
    """已知四元数应得到正确的 yaw"""
    # 绕 Z 轴 +90°：z=sin(45°)=√2/2, w=cos(45°)=√2/2
    s = math.sin(math.pi / 4)
    yaw = RosRobotNode._quaternion_to_yaw(0.0, 0.0, s, s)
    assert _angle_diff(yaw, math.pi / 2) < 1e-9

    # 单位四元数 (0,0,0,1) -> yaw=0
    yaw0 = RosRobotNode._quaternion_to_yaw(0.0, 0.0, 0.0, 1.0)
    assert _angle_diff(yaw0, 0.0) < 1e-9
