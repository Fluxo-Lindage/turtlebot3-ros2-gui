"""
ROS2 通信节点模块

封装了与 ROS2 的所有通信：
- 订阅 /odom 获取里程计（线速度和角速度）
- 通过 TF 实时查询机器人在地图(map)坐标系中的真实位姿
- 订阅 /map 获取实时占据栅格地图
- 通过 NavigateToPose action 发送导航目标
- 发布初始位姿用于 AMCL 定位
"""

import math
import threading
import time
import numpy as np
from typing import Optional, Tuple

from PyQt5.QtCore import QThread, pyqtSignal, QObject
from PyQt5.QtGui import QImage

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time as RosTime
from rclpy.duration import Duration as RosDuration
from rclpy.qos import qos_profile_sensor_data

# ROS2 消息和动作类型
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import (
    PoseWithCovarianceStamped, PoseStamped,
    Point, Quaternion, TransformStamped,
)
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image as SensorImage
from tf2_ros import Buffer, TransformListener

from .gazebo_view import image_msg_to_qimage


class RosRobotNode(Node):
    """
    机器人控制 ROS2 节点

    核心改进 (v2.1):
      - 使用 TF (tf2_ros) 将机器人位姿从 odom 坐标系
        实时转换到 map 坐标系，确保地图上显示的位置正确。
      - 对 OccupancyGrid 数据做 np.flipud() 翻转，
        使数据行序与 QImage 渲染方向一致。
    """

    def __init__(self):
        super().__init__('robot_gui_node')

        # ---- TF2 缓冲区和监听器 (用于 odom→map 坐标转换) ----
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ---------- 订阅器 ----------
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, 10
        )

        # ---------- 发布器 ----------
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )

        # ---------- Action 客户端 ----------
        self.nav_action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        self._current_goal_handle = None

        # ---------- 线程安全数据锁 ----------
        self._lock = threading.Lock()

        # 里程计原始数据（odom 坐标系）
        self._odom_x: float = 0.0
        self._odom_y: float = 0.0
        self._odom_yaw: float = 0.0
        self._linear_vel: float = 0.0
        self._angular_vel: float = 0.0
        self._odom_received = False
        self._last_odom_time = 0.0

        # 地图坐标系下的机器人位姿（通过 TF 计算）
        self._map_x: float = 0.0
        self._map_y: float = 0.0
        self._map_yaw: float = 0.0

        # 地图数据（已翻转，使 row 0 = 顶部，与 QImage 一致）
        self._map_data: Optional[np.ndarray] = None
        self._map_resolution: float = 0.05
        self._map_origin_x: float = 0.0
        self._map_origin_y: float = 0.0
        self._map_width: int = 0
        self._map_height: int = 0
        self._map_received = False
        self._map_seq: int = -1

        # 待上报给 GUI 的导航事件 [(success, message), ...]
        # Action 回调在 executor 线程触发，通过此队列交给 RosSpinThread 用信号发出
        self._nav_results: list = []

        # ---- GUI 嵌入式相机 ----
        # 自动发现 /gui_camera/.../image_raw 话题并订阅（话题名不写死，避免猜错）
        self._gui_cam_sub = None
        self._gui_cam_image: Optional[QImage] = None
        self._gui_cam_seq: int = -1

        self.get_logger().info('ROS 机器人节点初始化完成 (含 TF 支持)')

    # ===================== 回调 =====================

    def _odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        twist = msg.twist.twist

        qx, qy, qz, qw = (
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w
        )
        yaw = self._quaternion_to_yaw(qx, qy, qz, qw)

        with self._lock:
            self._odom_x = pose.position.x
            self._odom_y = pose.position.y
            self._odom_yaw = yaw
            self._linear_vel = twist.linear.x
            self._angular_vel = twist.angular.z
            self._odom_received = True
            self._last_odom_time = time.time()

    def _map_callback(self, msg: OccupancyGrid):
        """
        地图数据回调

        **重要**：ROS OccupancyGrid 中 row 0 = 地图底部（y 最小处），
        但 QImage 中 row 0 = 图片顶部。这里用 np.flipud() 翻转，
        保证 row 0 → 图片顶部，坐标计算正确。
        """
        with self._lock:
            h, w = msg.info.height, msg.info.width
            data = np.array(msg.data, dtype=np.int8).reshape(h, w)
            # **翻转**：使 row 0 对应地图顶部（y 最大处），与 QImage 一致
            data = np.flipud(data)

            self._map_data = data
            self._map_resolution = msg.info.resolution
            self._map_origin_x = msg.info.origin.position.x
            self._map_origin_y = msg.info.origin.position.y
            self._map_width = w
            self._map_height = h
            self._map_received = True
            self._map_seq += 1

    # ===================== 公共接口 =====================

    def get_robot_state(self) -> dict:
        """
        获取机器人状态（map 坐标系 + odom 速度）

        通过 TF 将 odom 中的位姿转换到 map 坐标系。
        若 TF 查询失败（例如 map 帧尚不存在），回退到 odom 坐标。
        """
        with self._lock:
            lv = self._linear_vel
            av = self._angular_vel
            # odom 超过 2 秒无新数据即视为断连
            now = time.time()
            odom_ok = self._odom_received and (now - self._last_odom_time < 2.0)
            odom_x = self._odom_x
            odom_y = self._odom_y
            odom_yaw = self._odom_yaw

        # ---- TF 查询：map ← base_link ----
        map_x, map_y, map_yaw = 0.0, 0.0, 0.0
        tf_ok = False
        try:
            # 查询最新的 map→base_link 变换
            trans: TransformStamped = self._tf_buffer.lookup_transform(
                'map', 'base_link', RosTime()
            )
            map_x = trans.transform.translation.x
            map_y = trans.transform.translation.y
            q = trans.transform.rotation
            map_yaw = self._quaternion_to_yaw(q.x, q.y, q.z, q.w)
            tf_ok = True

            with self._lock:
                self._map_x = map_x
                self._map_y = map_y
                self._map_yaw = map_yaw

        except Exception:
            # TF 尚不可用 → 使用上一次 TF 结果或回退到 odom
            with self._lock:
                if self._map_x == 0.0 and self._map_y == 0.0:
                    # 回退：直接使用 odom 坐标（SLAM 初期 map≈odom）
                    map_x = self._odom_x
                    map_y = self._odom_y
                    map_yaw = self._odom_yaw
                else:
                    map_x = self._map_x
                    map_y = self._map_y
                    map_yaw = self._map_yaw

        return {
            'x': map_x,
            'y': map_y,
            'yaw': map_yaw,
            'odom_x': odom_x,
            'odom_y': odom_y,
            'odom_yaw': odom_yaw,
            'linear_vel': lv,
            'angular_vel': av,
            'odom_ok': odom_ok,
            'tf_ok': tf_ok,
        }

    def reset_state(self):
        """完全重置机器人状态和地图缓存（切换仿真环境时调用）"""
        with self._lock:
            self._odom_received = False
            self._last_odom_time = 0.0
            self._odom_x = 0.0
            self._odom_y = 0.0
            self._odom_yaw = 0.0
            self._linear_vel = 0.0
            self._angular_vel = 0.0
            self._map_x = 0.0
            self._map_y = 0.0
            self._map_yaw = 0.0
            self._map_data = None
            self._map_received = False
            self._map_seq += 1
            self._map_resolution = 0.05
            self._map_origin_x = 0.0
            self._map_origin_y = 0.0
            self._map_width = 0
            self._map_height = 0

    def reset_map(self):
        """清除缓存的地图数据（切换仿真环境时调用）"""
        with self._lock:
            self._map_data = None
            self._map_received = False
            self._map_seq += 1
            self._map_resolution = 0.05
            self._map_origin_x = 0.0
            self._map_origin_y = 0.0
            self._map_width = 0
            self._map_height = 0

    def get_map_data(self) -> Optional[dict]:
        """获取最新地图数据（已翻转，可直接渲染）"""
        with self._lock:
            if not self._map_received or self._map_data is None:
                return None
            return {
                'data': self._map_data.copy(),
                'resolution': self._map_resolution,
                'origin_x': self._map_origin_x,
                'origin_y': self._map_origin_y,
                'width': self._map_width,
                'height': self._map_height,
                'seq': self._map_seq,
            }

    def send_navigation_goal(self, x: float, y: float, yaw: float = 0.0) -> bool:
        if not self.nav_action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('导航 Action Server 未就绪！请先启动导航。')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position = Point(x=x, y=y, z=0.0)
        goal_msg.pose.pose.orientation = self._yaw_to_quaternion(yaw)

        self.get_logger().info(f'发送导航目标: ({x:.2f}, {y:.2f})')

        send_goal_future = self.nav_action_client.send_goal_async(
            goal_msg, feedback_callback=self._nav_feedback_callback
        )
        send_goal_future.add_done_callback(self._nav_goal_response_callback)
        return True

    def cancel_navigation(self):
        if self._current_goal_handle is not None:
            self.get_logger().info('取消当前导航任务')
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None

    def publish_initial_pose(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position = Point(x=x, y=y, z=0.0)
        msg.pose.pose.orientation = self._yaw_to_quaternion(yaw)
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0685,
        ]
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f'已发布初始位姿: ({x:.2f}, {y:.2f})')

    # ===================== Action 回调 =====================

    def _nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('导航目标被拒绝！')
            with self._lock:
                self._nav_results.append((False, '导航目标被拒绝！'))
            return
        self.get_logger().info('导航目标已被接受')
        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_feedback_callback(self, feedback_msg):
        pass

    def _nav_result_callback(self, future):
        result = future.result()
        if result.result == 0:
            msg = '导航完成！已到达目标点。'
            self.get_logger().info(msg)
            with self._lock:
                self._nav_results.append((True, msg))
        else:
            msg = f'导航结束，状态码: {result.result}'
            self.get_logger().warn(msg)
            with self._lock:
                self._nav_results.append((False, msg))
        self._current_goal_handle = None

    def take_nav_results(self) -> list:
        """取出并清空待上报的导航事件（由 RosSpinThread 调用）"""
        with self._lock:
            results = self._nav_results[:]
            self._nav_results.clear()
            return results

    # ===================== GUI 嵌入式相机 =====================

    def discover_gui_camera_topic(self) -> bool:
        """
        自动查找 gui_camera 的 image_raw 话题并订阅。

        话题名不写死：spawn 出来的相机实际话题可能是
        /gui_camera/cam/image_raw 或 /gui_camera/image_raw，
        这里用 get_topic_names_and_types() 发现后按规则匹配。
        已订阅则直接返回 True。
        """
        if self._gui_cam_sub is not None:
            return True
        for name, types in self.get_topic_names_and_types():
            if 'gui_camera' not in name:
                continue
            if 'image_raw' not in name:
                continue
            if 'sensor_msgs/msg/Image' not in (types or []):
                continue
            self._subscribe_gui_camera(name)
            return True
        return False

    def _subscribe_gui_camera(self, topic: str):
        self._gui_cam_sub = self.create_subscription(
            SensorImage, topic, self._gui_camera_callback, qos_profile_sensor_data
        )
        self.get_logger().info(f'已订阅 GUI 相机: {topic}')

    def _gui_camera_callback(self, msg):
        img = image_msg_to_qimage(msg)
        if img.isNull():
            return
        with self._lock:
            self._gui_cam_image = img
            self._gui_cam_seq += 1

    def get_gui_camera_image(self) -> Tuple[Optional[QImage], int]:
        """返回 (最新 QImage, seq)；无数据时返回 (None, seq)"""
        with self._lock:
            return self._gui_cam_image, self._gui_cam_seq

    def clear_gui_camera(self):
        """清除缓存图片并重置订阅（停止/切换仿真时调用，便于下次重新发现）"""
        with self._lock:
            self._gui_cam_image = None
            self._gui_cam_seq += 1
        if self._gui_cam_sub is not None:
            try:
                self.destroy_subscription(self._gui_cam_sub)
            except Exception:
                pass
            self._gui_cam_sub = None

    # ===================== 数学工具 =====================

    @staticmethod
    def _quaternion_to_yaw(qx, qy, qz, qw):
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _yaw_to_quaternion(yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


class RosSpinThread(QThread):
    """ROS2 后台运行线程（与 v2.0 一致）"""

    state_updated = pyqtSignal(dict)
    map_updated = pyqtSignal(dict)
    nav_result = pyqtSignal(bool, str)  # (success, message) 导航完成/拒绝反馈
    gazebo_image_updated = pyqtSignal(QImage)  # GUI 相机最新一帧

    def __init__(self, ros_node: RosRobotNode, parent=None):
        super().__init__(parent)
        self._ros_node = ros_node
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(ros_node)
        self._running = False
        self._state_interval = 0.1   # 100ms
        self._map_interval = 0.5     # 500ms
        self._cam_discover_interval = 2.0  # 相机话题发现
        self._cam_img_interval = 0.1        # 相机帧轮询
        self._last_map_seq = -1
        self._last_gui_img_seq = -1

    def run(self):
        self._running = True
        next_state = time.time()
        next_map = time.time()
        next_cam_discover = time.time()
        next_cam_img = time.time()

        while self._running:
            self._executor.spin_once(timeout_sec=0.05)
            now = time.time()

            if now >= next_state:
                state = self._ros_node.get_robot_state()
                self.state_updated.emit(state)
                next_state = now + self._state_interval

            if now >= next_cam_discover:
                # 相机由 process_manager 异步注入，话题出现后自动订阅
                self._ros_node.discover_gui_camera_topic()
                next_cam_discover = now + self._cam_discover_interval

            if now >= next_cam_img:
                img, seq = self._ros_node.get_gui_camera_image()
                if img is not None and seq != self._last_gui_img_seq:
                    self._last_gui_img_seq = seq
                    self.gazebo_image_updated.emit(img)
                next_cam_img = now + self._cam_img_interval

            if now >= next_map:
                map_data = self._ros_node.get_map_data()
                if map_data is not None and map_data['seq'] != self._last_map_seq:
                    self._last_map_seq = map_data['seq']
                    self.map_updated.emit(map_data)
                next_map = now + self._map_interval

            # 转发导航事件（Action 回调在 executor 线程产生，此处转到 GUI 线程）
            for success, msg in self._ros_node.take_nav_results():
                self.nav_result.emit(success, msg)

        self._executor.remove_node(self._ros_node)

    def stop(self):
        self._running = False
        self.wait(timeout=3000)

    @property
    def ros_node(self):
        return self._ros_node
