"""
主界面模块 (v2.4 — 米黄色简化 UI)

布局:
  +-----------+-----------------------+-----------+
  |  控制面板  |       实时地图         | 机器人状态  |
  +-----------+-----------------------+-----------+
  |  系统日志                                        |
  +--------------------------------------------------+
"""

import math
import os
import datetime
import time
from typing import Optional

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QPushButton, QLabel, QLineEdit,
    QRadioButton, QButtonGroup, QTextEdit,
    QSplitter, QFrame, QGridLayout, QSizePolicy,
    QMessageBox, QApplication, QFileDialog, QScrollArea,
    QTabWidget,
)
from PyQt5.QtCore import Qt, QTimer, pyqtSlot
from PyQt5.QtGui import QFont, QColor, QTextCursor

import rclpy
from rclpy.executors import MultiThreadedExecutor
import numpy as np

from .ros_node import RosRobotNode, RosSpinThread
from .process_manager import ProcessManager, AVAILABLE_WORLDS, ProcessType
from .gazebo_view import GazeboViewWidget
from .map_widget import MapWidget

DEFAULT_MAP_DIR = os.path.expanduser('~/robot_maps')


class MainWindow(QMainWindow):

    MAX_LOG_BLOCKS = 500  # 日志面板保留的最大行数，超出则从最旧行开始删除

    def __init__(self, parent=None):
        super().__init__(parent)

        self._ros_node: Optional[RosRobotNode] = None
        self._ros_thread: Optional[RosSpinThread] = None
        self._process_manager = ProcessManager()

        self._current_map_path = ''
        self._current_planner = 'dwa'
        self._gazebo_running = False
        self._slam_running = False
        self._nav_running = False

        self._last_map_render_time = 0.0

        self._init_ui()
        self._init_ros()
        self._connect_signals()

        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._update_status_indicators)
        self._status_timer.start(500)

        self._refresh_saved_maps()
        QApplication.instance().aboutToQuit.connect(self._cleanup)

    # ===================== UI 构建 =====================

    def _init_ui(self):
        self.setWindowTitle('机器人仿真控制平台 — TurtleBot3 + ROS2 Humble')
        self.setMinimumSize(1200, 750)
        self._apply_style()

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(8)
        main_layout.setContentsMargins(10, 10, 10, 10)

        main_layout.addWidget(self._create_top_bar())

        body = QSplitter(Qt.Horizontal)

        left = QScrollArea()
        left.setWidgetResizable(True)
        left.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        left.setWidget(self._create_control_panel())
        left.setMinimumWidth(280)
        left.setMaximumWidth(380)
        body.addWidget(left)

        self._map_widget = MapWidget()
        self._map_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self._gazebo_view = GazeboViewWidget()
        self._gazebo_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # 中间用 Tab 切换“实时地图”和“仿真视图（Gazebo 相机）”
        self._view_tabs = QTabWidget()
        self._view_tabs.addTab(self._map_widget, '实时地图')
        self._view_tabs.addTab(self._gazebo_view, '仿真视图')
        body.addWidget(self._view_tabs)

        right = self._create_robot_state_panel()
        right.setMinimumWidth(240)
        right.setMaximumWidth(350)
        body.addWidget(right)

        body.setSizes([300, 600, 260])
        main_layout.addWidget(body, 1)

        main_layout.addWidget(self._create_log_panel())

    def _apply_style(self):
        self.setStyleSheet("""
            QMainWindow { background-color: #f5f0e1; }
            QGroupBox {
                color: #3d3226; font-weight: bold; font-size: 16px;
                border: 1px solid #c4b896; border-radius: 8px;
                margin-top: 18px; padding-top: 18px;
                background-color: #ede6d5;
            }
            QGroupBox::title {
                subcontrol-origin: margin; left: 12px; padding: 0 6px;
                color: #6b5c4a;
            }
            QPushButton {
                background-color: #d4c9ad; color: #3d3226;
                border: 1px solid #b8a98c; border-radius: 6px;
                padding: 8px 16px; font-size: 15px; min-height: 34px;
            }
            QPushButton:hover { background-color: #c4b896; }
            QPushButton:pressed { background-color: #b8a98c; }
            QPushButton:disabled { background-color: #e8e0cf; color: #b8a98c; }
            QPushButton#btn_stop_all {
                background-color: #c94b3d; color: #fff; font-weight: bold; font-size: 16px;
            }
            QPushButton#btn_stop_all:hover { background-color: #d55c4e; }
            QPushButton.preset-goal { font-size: 13px; padding: 4px 10px; min-height: 28px; }
            QRadioButton { color: #3d3226; font-size: 15px; }
            QLabel { color: #3d3226; font-size: 15px; }
            QLineEdit {
                background-color: #faf7ef; color: #3d3226;
                border: 1px solid #c4b896; border-radius: 4px;
                padding: 5px 10px; font-size: 15px;
            }
            QTextEdit {
                background-color: #faf7ef; color: #3d3226;
                border: 1px solid #c4b896; border-radius: 4px;
                font-family: 'Monospace'; font-size: 14px;
            }
            QScrollArea { border: none; background: transparent; }
            QScrollBar:vertical { background: #ede6d5; width: 8px; border-radius: 4px; }
            QScrollBar::handle:vertical { background: #c4b896; border-radius: 4px; min-height: 30px; }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
            QSplitter::handle { background-color: #c4b896; width: 3px; height: 3px; }
        """)

    def _create_top_bar(self):
        bar = QWidget(); bar.setFixedHeight(50)
        layout = QHBoxLayout(bar); layout.setContentsMargins(4, 2, 4, 2)
        t = QLabel('机器人仿真控制平台')
        f = QFont('Sans', 20); f.setBold(True); t.setFont(f)
        t.setStyleSheet('color:#6b5c4a; border:none; background:transparent;'); layout.addWidget(t)
        layout.addStretch()
        self._status_labels = {}
        for k, txt in [('gazebo', 'Gazebo'), ('slam', 'SLAM'), ('navigation', '导航'), ('robot', '机器人')]:
            l = QLabel(txt)
            lf = QFont('Sans', 15); lf.setBold(True); l.setFont(lf)
            l.setStyleSheet('color:#6b5c4a; border:none; background:transparent; padding:2px 10px;')
            self._status_labels[k] = l; layout.addWidget(l)
        return bar

    # ---- 左侧控制面板 ----

    def _create_control_panel(self):
        p = QWidget(); ly = QVBoxLayout(p); ly.setSpacing(8); ly.setContentsMargins(4, 4, 4, 4)
        ly.addWidget(self._create_simulation_group())
        ly.addWidget(self._create_slam_group())
        ly.addWidget(self._create_navigation_group())
        ly.addWidget(self._create_planner_group())
        b = QPushButton('一键停止所有'); b.setObjectName('btn_stop_all')
        b.clicked.connect(self._on_stop_all); ly.addWidget(b)
        ly.addStretch(); return p

    def _create_simulation_group(self):
        g = QGroupBox('仿真环境'); ly = QVBoxLayout(g); ly.setSpacing(6)
        self._world_radios = {}; self._world_btn_group = QButtonGroup(self)
        for i, (k, info) in enumerate(AVAILABLE_WORLDS.items()):
            r = QRadioButton(info['name']); r.setToolTip(info['description'])
            if i == 0: r.setChecked(True)
            self._world_radios[k] = r; self._world_btn_group.addButton(r, i); ly.addWidget(r)
        ly.addSpacing(4)
        br = QHBoxLayout()
        self._btn_gazebo_start = QPushButton('启动仿真')
        self._btn_gazebo_start.clicked.connect(self._on_start_gazebo); br.addWidget(self._btn_gazebo_start)
        self._btn_gazebo_stop = QPushButton('停止'); self._btn_gazebo_stop.setEnabled(False)
        self._btn_gazebo_stop.clicked.connect(self._on_stop_gazebo); br.addWidget(self._btn_gazebo_stop)
        ly.addLayout(br)
        self._btn_respawn_cam = QPushButton('重新生成相机')
        self._btn_respawn_cam.setToolTip('向 Gazebo 重新注入 GUI 仿真相机（用于首启失败或切换场景后）')
        self._btn_respawn_cam.clicked.connect(self._on_respawn_cam); ly.addWidget(self._btn_respawn_cam)
        return g

    def _create_slam_group(self):
        g = QGroupBox('SLAM 建图'); ly = QVBoxLayout(g); ly.setSpacing(6)
        br = QHBoxLayout()
        self._btn_slam_start = QPushButton('开始建图'); self._btn_slam_start.clicked.connect(self._on_start_slam); br.addWidget(self._btn_slam_start)
        self._btn_slam_stop = QPushButton('停止'); self._btn_slam_stop.setEnabled(False)
        self._btn_slam_stop.clicked.connect(self._on_stop_slam); br.addWidget(self._btn_slam_stop)
        ly.addLayout(br)
        self._btn_save_map = QPushButton('保存地图'); self._btn_save_map.clicked.connect(self._on_save_map); ly.addWidget(self._btn_save_map)
        self._btn_teleop = QPushButton('打开键盘遥控器'); self._btn_teleop.clicked.connect(self._on_launch_teleop); ly.addWidget(self._btn_teleop)
        self._lbl_saved_maps = QLabel('(暂无已保存地图)'); self._lbl_saved_maps.setStyleSheet('color:#8b7355; font-size:12px; border:none; background:transparent;'); self._lbl_saved_maps.setWordWrap(True); ly.addWidget(self._lbl_saved_maps)
        return g

    def _create_navigation_group(self):
        g = QGroupBox('自主导航'); ly = QVBoxLayout(g); ly.setSpacing(6)
        self._btn_load_map = QPushButton('加载地图'); self._btn_load_map.clicked.connect(self._on_load_map); ly.addWidget(self._btn_load_map)
        self._lbl_current_map = QLabel('当前地图: (未加载)'); self._lbl_current_map.setStyleSheet('color:#8b7355; font-size:13px; border:none; background:transparent;'); self._lbl_current_map.setWordWrap(True); ly.addWidget(self._lbl_current_map)
        br = QHBoxLayout()
        self._btn_nav_start = QPushButton('开始导航'); self._btn_nav_start.setEnabled(False)
        self._btn_nav_start.clicked.connect(self._on_start_nav); br.addWidget(self._btn_nav_start)
        self._btn_nav_stop = QPushButton('停止'); self._btn_nav_stop.setEnabled(False)
        self._btn_nav_stop.clicked.connect(self._on_stop_nav); br.addWidget(self._btn_nav_stop)
        ly.addLayout(br)
        self._btn_cancel_goal = QPushButton('取消当前导航目标'); self._btn_cancel_goal.setEnabled(False)
        self._btn_cancel_goal.clicked.connect(self._on_cancel_goal); ly.addWidget(self._btn_cancel_goal)
        cl = QLabel('或手动输入:'); cl.setStyleSheet('color:#8b7355; font-size:13px; border:none; background:transparent;'); ly.addWidget(cl)
        cg = QGridLayout()
        cg.addWidget(QLabel('X:'), 0, 0); self._edit_goal_x = QLineEdit('0.0'); self._edit_goal_x.setMaximumWidth(80); cg.addWidget(self._edit_goal_x, 0, 1)
        cg.addWidget(QLabel('Y:'), 0, 2); self._edit_goal_y = QLineEdit('0.0'); self._edit_goal_y.setMaximumWidth(80); cg.addWidget(self._edit_goal_y, 0, 3)
        ly.addLayout(cg)
        br2 = QHBoxLayout()
        self._btn_send_goal = QPushButton('发送'); self._btn_send_goal.setEnabled(False)
        self._btn_send_goal.clicked.connect(self._on_send_goal); br2.addWidget(self._btn_send_goal)
        for nm, px, py in [('原点', 0, 0), ('(2,0)', 2, 0), ('(0,1)', 0, 1), ('(-2,0)', -2, 0)]:
            b = QPushButton(nm); b.setProperty('class', 'preset-goal')
            b.clicked.connect(lambda _, x=px, y=py: self._set_goal_preset(x, y)); br2.addWidget(b)
        ly.addLayout(br2); return g

    def _create_planner_group(self):
        g = QGroupBox('避障算法'); ly = QVBoxLayout(g); ly.setSpacing(6)
        self._planner_btn_group = QButtonGroup(self)
        self._radio_dwa = QRadioButton('DWA (动态窗口法)'); self._radio_dwa.setChecked(True)
        self._planner_btn_group.addButton(self._radio_dwa, 0); ly.addWidget(self._radio_dwa)
        self._radio_teb = QRadioButton('TEB (时间弹性带)')
        self._planner_btn_group.addButton(self._radio_teb, 1); ly.addWidget(self._radio_teb)
        self._btn_apply_planner = QPushButton('应用算法'); self._btn_apply_planner.clicked.connect(self._on_apply_planner); ly.addWidget(self._btn_apply_planner)
        return g

    def _create_robot_state_panel(self):
        p = QWidget(); ly = QVBoxLayout(p); ly.setSpacing(8); ly.setContentsMargins(4, 4, 4, 4)
        pg = QGroupBox('位置'); pl = QGridLayout(pg); pl.setSpacing(8)
        pl.addWidget(QLabel('X:'), 0, 0); self._lbl_pos_x = self._vlbl('0.000 m'); pl.addWidget(self._lbl_pos_x, 0, 1)
        pl.addWidget(QLabel('Y:'), 1, 0); self._lbl_pos_y = self._vlbl('0.000 m'); pl.addWidget(self._lbl_pos_y, 1, 1)
        pl.addWidget(QLabel('朝向:'), 2, 0); self._lbl_yaw = self._vlbl('0.0'); pl.addWidget(self._lbl_yaw, 2, 1)
        ly.addWidget(pg)
        vg = QGroupBox('速度'); vl = QGridLayout(vg); vl.setSpacing(8)
        vl.addWidget(QLabel('线速度:'), 0, 0); self._lbl_linear_vel = self._vlbl('0.000 m/s'); vl.addWidget(self._lbl_linear_vel, 0, 1)
        vl.addWidget(QLabel('角速度:'), 1, 0); self._lbl_angular_vel = self._vlbl('0.000 rad/s'); vl.addWidget(self._lbl_angular_vel, 1, 1)
        ly.addWidget(vg)
        sg = QGroupBox('状态'); sl = QGridLayout(sg); sl.setSpacing(8)
        sl.addWidget(QLabel('里程计:'), 0, 0); self._lbl_odom_status = self._vlbl('等待...', '#8b7355'); sl.addWidget(self._lbl_odom_status, 0, 1)
        sl.addWidget(QLabel('算法:'), 1, 0); self._lbl_planner = self._vlbl('DWA', '#5a7d4a'); sl.addWidget(self._lbl_planner, 1, 1)
        sl.addWidget(QLabel('地图:'), 2, 0); self._lbl_map_status = self._vlbl('等待...', '#8b7355'); sl.addWidget(self._lbl_map_status, 2, 1)
        ly.addWidget(sg)
        tg = QGroupBox('操作'); tl = QVBoxLayout(tg)
        tp = QLabel('左键:设目标  右键:清除\n滚轮:缩放  拖动:平移\n双击:适应  R:重置')
        tp.setStyleSheet('color:#8b7355; font-size:13px; border:none; background:transparent;'); tl.addWidget(tp); ly.addWidget(tg)
        ly.addStretch(); return p

    def _create_log_panel(self):
        g = QGroupBox('系统日志'); ly = QVBoxLayout(g); ly.setContentsMargins(6, 4, 6, 6)
        self._log_text = QTextEdit(); self._log_text.setReadOnly(True); self._log_text.setMaximumHeight(130)
        ly.addWidget(self._log_text)
        b = QPushButton('清空'); b.setMaximumWidth(60)
        b.setStyleSheet('font-size:13px; padding:2px 6px; min-height:24px;'); b.clicked.connect(self._log_text.clear); ly.addWidget(b)
        return g

    # ===================== 信号 & ROS =====================

    def _connect_signals(self):
        pm = self._process_manager
        pm.log_message.connect(self._on_log_message)
        pm.process_started.connect(self._on_process_started)
        pm.process_stopped.connect(self._on_process_stopped)
        pm.map_saved.connect(self._on_map_saved)
        self._map_widget.goal_selected.connect(self._on_map_goal_selected)

    def _init_ros(self):
        self._process_manager.cleanup_leftovers()
        rclpy.init(args=[])
        self._ros_node = RosRobotNode()
        self._ros_thread = RosSpinThread(self._ros_node)
        self._ros_thread.state_updated.connect(self._on_state_updated)
        self._ros_thread.map_updated.connect(self._on_map_updated)
        self._ros_thread.nav_result.connect(self._on_nav_result)
        self._ros_thread.gazebo_image_updated.connect(self._gazebo_view.update_image)
        self._ros_thread.start()
        self._log('info', 'ROS2 通信节点已启动')

    # ===================== 槽函数 =====================

    def _on_start_gazebo(self):
        world_key = 'world'
        for key, radio in self._world_radios.items():
            if radio.isChecked(): world_key = key; break
        if self._process_manager.start_gazebo(world_key):
            self._btn_gazebo_start.setEnabled(False); self._btn_gazebo_stop.setEnabled(True)
            self._gazebo_running = True
            # 异步注入 GUI 仿真相机（失败会在日志里提示，可点“重新生成相机”重试）
            self._process_manager.spawn_gui_camera()

    def _on_respawn_cam(self):
        if not self._gazebo_running:
            QMessageBox.information(self, '提示', '请先启动仿真环境！'); return
        if self._ros_node: self._ros_node.clear_gui_camera()
        self._gazebo_view.clear_view()
        self._process_manager.spawn_gui_camera()

    def _on_stop_gazebo(self):
        self._process_manager.stop_gazebo()
        self._btn_gazebo_start.setEnabled(True); self._btn_gazebo_stop.setEnabled(False)
        self._gazebo_running = False
        self._clear_map_and_state()

    def _on_start_slam(self):
        if not self._gazebo_running: QMessageBox.warning(self, '提示', '请先启动仿真环境！'); return
        if self._process_manager.start_slam():
            self._btn_slam_start.setEnabled(False); self._btn_slam_stop.setEnabled(True)
            self._slam_running = True
            # 新建图开始：重置 map 坐标位姿缓存，避免残留旧坐标导致显示卡住
            if self._ros_node: self._ros_node.reset_map_pose()
            self._map_widget.clear_robot_trail()

    def _on_stop_slam(self):
        self._process_manager.stop_slam()
        self._btn_slam_start.setEnabled(True); self._btn_slam_stop.setEnabled(False)
        self._slam_running = False

    def _on_save_map(self):
        if not self._slam_running: QMessageBox.warning(self, '提示', '请先启动 SLAM 建图！'); return
        os.makedirs(DEFAULT_MAP_DIR, exist_ok=True)
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        path, _ = QFileDialog.getSaveFileName(self, '保存地图', os.path.join(DEFAULT_MAP_DIR, f'map_{ts}'), 'YAML (*.yaml)')
        if not path: return
        if path.endswith('.yaml'): path = path[:-5]
        # 异步保存：保存期间禁用按钮，完成后由 map_saved 信号刷新列表
        self._btn_save_map.setEnabled(False)
        self._btn_save_map.setText('保存中...')
        self._process_manager.save_map(path)

    @pyqtSlot(bool, str)
    def _on_map_saved(self, success: bool, path: str):
        self._btn_save_map.setEnabled(True)
        self._btn_save_map.setText('保存地图')
        if success:
            self._refresh_saved_maps()

    def _on_launch_teleop(self):
        self._process_manager.launch_teleop_terminal()

    def _refresh_saved_maps(self):
        os.makedirs(DEFAULT_MAP_DIR, exist_ok=True)
        files = sorted([f for f in os.listdir(DEFAULT_MAP_DIR) if f.endswith('.yaml')])
        self._lbl_saved_maps.setText(('已保存: ' + ', '.join(files[-3:]) + (f' ...等{len(files)}个' if len(files) > 3 else '')) if files else '(暂无)')

    def _on_load_map(self):
        os.makedirs(DEFAULT_MAP_DIR, exist_ok=True)
        path, _ = QFileDialog.getOpenFileName(self, '选择地图', DEFAULT_MAP_DIR, 'YAML (*.yaml)')
        if not path: return
        self._current_map_path = path
        self._btn_nav_start.setEnabled(True)
        self._lbl_current_map.setText(f'当前: {os.path.basename(path)}')
        self._log('info', f'已加载: {os.path.basename(path)}')

    def _on_start_nav(self):
        if not self._current_map_path: QMessageBox.warning(self, '提示', '请先加载地图！'); return
        if not self._gazebo_running: QMessageBox.warning(self, '提示', '请先启动仿真！'); return
        if self._slam_running:
            self._process_manager.stop_slam()
            self._btn_slam_start.setEnabled(True); self._btn_slam_stop.setEnabled(False)
            self._slam_running = False
            self._log('info', '已自动停止 SLAM（避免与导航地图冲突）')
        p = self._current_planner
        if self._process_manager.start_navigation(self._current_map_path, p):
            self._btn_nav_start.setEnabled(False); self._btn_nav_stop.setEnabled(True)
            self._btn_send_goal.setEnabled(True); self._btn_cancel_goal.setEnabled(True)
            self._nav_running = True; self._lbl_planner.setText(p.upper())
            # 切地图后重置 map 坐标位姿缓存，避免回退到上一张地图的旧坐标导致机器人“卡住”
            if self._ros_node: self._ros_node.reset_map_pose()
            self._map_widget.clear_robot_trail()
            state = self._ros_node.get_robot_state() if self._ros_node else {}
            init_x = state.get('odom_x', 0.0)
            init_y = state.get('odom_y', 0.0)
            init_yaw = state.get('odom_yaw', 0.0)
            self._log('info', f'AMCL 初始位姿: ({init_x:.2f}, {init_y:.2f})')
            QTimer.singleShot(5000, lambda x=init_x, y=init_y, yaw=init_yaw:
                self._ros_node.publish_initial_pose(x, y, yaw) if self._ros_node else None)

    def _on_stop_nav(self):
        if self._ros_node: self._ros_node.cancel_navigation()
        self._map_widget.clear_goal(); self._process_manager.stop_navigation()
        self._btn_nav_start.setEnabled(True); self._btn_nav_stop.setEnabled(False)
        self._btn_send_goal.setEnabled(False); self._btn_cancel_goal.setEnabled(False)
        self._nav_running = False

    def _on_map_goal_selected(self, x, y, yaw):
        if not self._nav_running: QMessageBox.information(self, '提示', '请先启动导航！'); self._map_widget.clear_goal(); return
        self._edit_goal_x.setText(f'{x:.3f}'); self._edit_goal_y.setText(f'{y:.3f}')
        if self._ros_node and self._ros_node.send_navigation_goal(x, y, yaw):
            self._map_widget.set_goal(x, y, yaw); self._log('info', f'地图点击->导航: ({x:.2f},{y:.2f})')

    def _on_send_goal(self):
        if not self._ros_node or not self._nav_running: return
        try: x = float(self._edit_goal_x.text()); y = float(self._edit_goal_y.text())
        except ValueError: QMessageBox.warning(self, '错误', '坐标必须是数字！'); return
        if self._ros_node.send_navigation_goal(x, y, 0.0):
            self._map_widget.set_goal(x, y, 0.0); self._log('info', f'发送目标: ({x:.2f},{y:.2f})')

    def _on_cancel_goal(self):
        if self._ros_node: self._ros_node.cancel_navigation()
        self._map_widget.clear_goal(); self._log('info', '已取消导航')

    @pyqtSlot(bool, str)
    def _on_nav_result(self, success: bool, msg: str):
        # 导航完成/失败/被拒的反馈落到 GUI 日志面板
        self._log('info' if success else 'warn', msg)
        if success:
            # 到达目标：清除目标标记，表示本次任务结束
            self._map_widget.clear_goal()

    def _set_goal_preset(self, x, y):
        self._edit_goal_x.setText(f'{x:.1f}'); self._edit_goal_y.setText(f'{y:.1f}')

    def _on_stop_all(self):
        if self._ros_node: self._ros_node.cancel_navigation()
        self._map_widget.clear_goal(); self._process_manager.stop_all()
        self._btn_gazebo_start.setEnabled(True); self._btn_gazebo_stop.setEnabled(False)
        self._btn_slam_start.setEnabled(True); self._btn_slam_stop.setEnabled(False)
        self._btn_nav_start.setEnabled(bool(self._current_map_path))
        self._btn_nav_stop.setEnabled(False); self._btn_send_goal.setEnabled(False); self._btn_cancel_goal.setEnabled(False)
        self._gazebo_running = self._slam_running = self._nav_running = False
        self._clear_map_and_state()

    def _on_apply_planner(self):
        old = self._current_planner
        self._current_planner = 'dwa' if self._radio_dwa.isChecked() else 'teb'
        self._lbl_planner.setText(self._current_planner.upper())
        if self._current_planner == old: self._log('info', f'已在 {self._current_planner.upper()} 模式'); return
        self._log('info', f'切换算法: {self._current_planner.upper()}')
        if self._nav_running and self._current_map_path:
            if QMessageBox.question(self, '重启导航', f'重启以应用 {self._current_planner.upper()}？', QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes) == QMessageBox.Yes:
                self._process_manager.switch_planner(self._current_map_path, self._current_planner)

    # ===================== ROS 更新 =====================

    @pyqtSlot(dict)
    def _on_state_updated(self, s):
        self._lbl_pos_x.setText(f'{s["x"]:.3f} m'); self._lbl_pos_y.setText(f'{s["y"]:.3f} m')
        self._lbl_yaw.setText(f'{math.degrees(s["yaw"]):.1f} deg')
        self._lbl_linear_vel.setText(f'{s["linear_vel"]:.3f} m/s')
        self._lbl_angular_vel.setText(f'{s["angular_vel"]:.3f} rad/s')
        ok = s['odom_ok']
        self._lbl_odom_status.setText('正常' if ok else '等待...')
        self._lbl_odom_status.setStyleSheet(f'color:{"#5a7d4a" if ok else "#8b7355"}; font-size:18px; font-weight:bold; border:none; background:transparent;')
        self._map_widget.update_robot_pose(s['x'], s['y'], s['yaw'])

    @pyqtSlot(dict)
    def _on_map_updated(self, m):
        now = time.time()
        self._last_map_render_time = now
        QTimer.singleShot(300, lambda: self._render_map_if_stable(m, now))

    def _render_map_if_stable(self, m, timestamp):
        if timestamp < self._last_map_render_time:
            return
        self._map_widget.update_map(
            m['data'], m['resolution'], m['origin_x'], m['origin_y'],
            m['width'], m['height']
        )
        self._lbl_map_status.setText('就绪')
        self._lbl_map_status.setStyleSheet('color:#5a7d4a; font-size:18px; font-weight:bold; border:none; background:transparent;')

    def _update_status_indicators(self):
        go = self._process_manager.is_running(ProcessType.GAZEBO)
        so = self._process_manager.is_running(ProcessType.SLAM)
        no = self._process_manager.is_running(ProcessType.NAVIGATION)
        ro = self._ros_node and self._ros_node.get_robot_state()['odom_ok']
        for k, ok in [('gazebo', go), ('slam', so), ('navigation', no), ('robot', ro)]:
            self._set_status(k, ok)

    def _set_status(self, key, ok):
        """红绿灯状态指示 — 用文字颜色区分，不使用 emoji"""
        lbl = self._status_labels.get(key)
        if lbl:
            c = '#5a7d4a' if ok else '#c94b3d'
            lbl.setStyleSheet(f'color:{c}; border:none; background:transparent; padding:2px 10px; font-size:15px; font-weight:bold;')

    # ===================== 日志 / 清理 =====================

    @pyqtSlot(str, str)
    def _on_log_message(self, msg, lvl): self._log(lvl, msg)

    @pyqtSlot(str)
    def _on_process_started(self, t): self._log('info', f'[系统] {t} 启动')

    @pyqtSlot(str)
    def _on_process_stopped(self, t):
        self._log('info', f'[系统] {t} 停止')
        if t == 'gazebo': self._btn_gazebo_start.setEnabled(True); self._btn_gazebo_stop.setEnabled(False); self._gazebo_running = False; self._clear_map_and_state()
        elif t == 'slam': self._btn_slam_start.setEnabled(True); self._btn_slam_stop.setEnabled(False); self._slam_running = False
        elif t == 'navigation': self._btn_nav_start.setEnabled(True); self._btn_nav_stop.setEnabled(False); self._nav_running = False

    def _clear_map_and_state(self):
        if self._ros_node:
            self._ros_node.reset_state()
            self._ros_node.clear_gui_camera()
        self._map_widget.clear_map()
        self._gazebo_view.clear_view()
        self._lbl_map_status.setText('等待中...')
        self._lbl_map_status.setStyleSheet('color:#8b7355; font-size:18px; font-weight:bold; border:none; background:transparent;')

    @staticmethod
    def _vlbl(text, color='#5a7d4a'):
        l = QLabel(text)
        l.setStyleSheet(f'color:{color}; font-size:18px; font-weight:bold; border:none; background:transparent; padding:2px 8px;')
        l.setAlignment(Qt.AlignLeft | Qt.AlignVCenter); return l

    def _log(self, level, msg):
        if not hasattr(self, '_log_text') or self._log_text is None: return
        ts = datetime.datetime.now().strftime('%H:%M:%S')
        cm = {'info': '#6b5c4a', 'warn': '#a07a30', 'error': '#c94b3d'}
        lm = {'info': 'INFO ', 'warn': 'WARN ', 'error': 'ERROR'}
        self._log_text.append(f'<span style="color:#b8a98c">[{ts}]</span> <span style="color:{cm.get(level, "#6b5c4a")}">[{lm.get(level, "INFO")}]</span> {msg}')
        c = self._log_text.textCursor(); c.movePosition(QTextCursor.End); self._log_text.setTextCursor(c)
        # 超过上限时只删除最旧的若干行，保留最近的日志（不再整体清空）
        doc = self._log_text.document()
        excess = doc.blockCount() - self.MAX_LOG_BLOCKS
        if excess > 0:
            cur = QTextCursor(doc)
            cur.movePosition(QTextCursor.Start)
            cur.movePosition(QTextCursor.NextBlock, QTextCursor.KeepAnchor, excess)
            cur.removeSelectedText()

    def _cleanup(self):
        self._log('info', '正在关闭...')
        if self._status_timer: self._status_timer.stop()
        if self._ros_thread: self._ros_thread.stop()
        if self._ros_node: self._ros_node.destroy_node()
        self._process_manager.stop_all()
        if rclpy.ok(): rclpy.shutdown()
        self._log('info', '已安全退出')

    def closeEvent(self, event):
        self._cleanup(); event.accept()
