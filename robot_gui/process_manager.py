"""
进程管理器模块

负责通过 subprocess 启动和停止 ROS2 的各个子系统：
- Gazebo 仿真环境（支持不同 world 切换）
- SLAM 建图（使用 slam_toolbox）
- Navigation2 导航（支持 DWA / TEB 切换）
- 地图保存

所有子进程通过 Popen 管理，支持随时启动和终止。
"""

import os
import signal
import subprocess
import threading
import time
from enum import Enum
from typing import Optional, Dict, Callable, List

from PyQt5.QtCore import QObject, pyqtSignal


# ===================== 常量定义 =====================

# 可用的仿真世界列表
AVAILABLE_WORLDS = {
    'world': {
        'name': '标准障碍物场地',
        'world_file': 'turtlebot3_world.world',
        'launch_file': 'turtlebot3_world.launch.py',
        'description': '开放场地 + 圆柱障碍物，适合基础测试',
    },
    'house': {
        'name': '室内房屋场景',
        'world_file': 'turtlebot3_house.world',
        'launch_file': 'turtlebot3_house.launch.py',
        'description': '模拟室内家居环境，有墙壁和房间',
    },
    'empty': {
        'name': '空旷场地',
        'world_file': 'empty_world.world',
        'launch_file': 'empty_world.launch.py',
        'description': '完全空旷的平面，无障碍物',
    },
    'dqn_stage1': {
        'name': 'DQN 训练场 Stage1',
        'world_file': 'turtlebot3_dqn_stage1.world',
        'launch_file': 'turtlebot3_dqn_stage1.launch.py',
        'description': '方形围墙 + L 型障碍，适合导航测试',
    },
}

# TurtleBot3 型号（从环境变量读取，默认为 waffle_pi）
TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')


class ProcessType(Enum):
    """子系统进程类型枚举"""
    GAZEBO = 'gazebo'        # Gazebo 仿真环境
    SLAM = 'slam'             # SLAM 建图
    NAVIGATION = 'navigation' # Navigation2 导航
    RVIZ = 'rviz'             # RViz 可视化


class ProcessManager(QObject):
    """
    ROS2 子进程管理器

    负责启动/停止 Gazebo、SLAM、Navigation 等子系统。
    所有子进程在后台运行，通过信号通知 GUI 状态变化。

    用法示例:
        pm = ProcessManager()
        pm.start_gazebo('world')       # 启动标准场地仿真
        pm.start_slam()                 # 启动建图
        pm.stop_slam()                  # 停止建图
        pm.save_map('/path/to/map')    # 保存地图
        pm.start_navigation('/path/to/map')  # 启动导航
    """

    # ---------- 信号 ----------
    log_message = pyqtSignal(str, str)  # (message, level: info/warn/error)
    process_started = pyqtSignal(str)   # process_type
    process_stopped = pyqtSignal(str)   # process_type
    map_saved = pyqtSignal(bool, str)   # (success, save_path) 地图保存完成回调

    def __init__(self, parent: Optional[QObject] = None):
        super().__init__(parent)

        # 子进程字典: {ProcessType: subprocess.Popen}
        self._processes: Dict[ProcessType, subprocess.Popen] = {}

        # 当前使用的世界
        self._current_world: str = 'world'

    # ===================== 公共 API =====================

    def start_gazebo(self, world_key: str) -> bool:
        """
        Starting Gazebo 仿真环境

        Args:
            world_key: 世界标识符，'world' 或 'house'

        Returns:
            bool: 启动成功返回 True
        """
        if world_key not in AVAILABLE_WORLDS:
            self.log_message.emit(f'未知的仿真环境: {world_key}', 'error')
            return False

        # 先停止已有的 Gazebo（并强制杀掉残留进程）
        self._stop_process(ProcessType.GAZEBO)
        self._force_kill_gazebo()  # 确保 gzserver/gzclient 彻底退出

        world_info = AVAILABLE_WORLDS[world_key]
        self._current_world = world_key

        self.log_message.emit(f'正在启动仿真环境: {world_info["name"]}...', 'info')

        # 设置环境变量
        env = os.environ.copy()
        env['TURTLEBOT3_MODEL'] = TURTLEBOT3_MODEL

        # 使用 ros2 launch Starting Gazebo
        cmd = [
            'ros2', 'launch',
            'turtlebot3_gazebo',
            world_info['launch_file'],
        ]

        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=env,
                preexec_fn=os.setsid,  # 创建新的进程组，方便终止
                text=True,
                bufsize=1,
            )
            self._processes[ProcessType.GAZEBO] = proc
            self.process_started.emit('gazebo')
            self.log_message.emit(
                f'仿真环境已启动: {world_info["name"]} ({world_info["description"]})', 'info'
            )
            # 启动日志读取线程
            self._start_log_reader(proc, ProcessType.GAZEBO)
            return True
        except FileNotFoundError:
            self.log_message.emit(
                '未找到 ros2 命令，请确保已 source ROS2 环境。', 'error'
            )
            return False

    def stop_gazebo(self):
        """停止 Gazebo 仿真"""
        self._stop_process(ProcessType.GAZEBO)
        self.log_message.emit('仿真环境已停止', 'info')

    def spawn_gui_camera(self) -> bool:
        """
        向运行中的 Gazebo 世界注入一个固定角度相机，供 GUI 嵌入显示。

        通过 ros2 run gazebo_ros spawn_entity.py 注入 gui_camera.sdf。
        异步执行（不阻塞 GUI）：先等 gzserver 就绪，失败则重试若干次。
        相机发布 /gui_camera/.../image_raw，由 ros_node 自动发现并订阅。

        Returns:
            bool: 任务已启动返回 True
        """
        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        sdf = os.path.join(pkg_dir, 'robot_gui', 'gazebo', 'gui_camera.sdf')
        if not os.path.exists(sdf):
            self.log_message.emit(f'相机 SDF 不存在: {sdf}', 'error')
            return False

        env = os.environ.copy()
        env['TURTLEBOT3_MODEL'] = TURTLEBOT3_MODEL

        def worker():
            # 等 gzserver + 世界加载完
            time.sleep(5.0)
            for attempt in range(5):
                try:
                    result = subprocess.run(
                        [
                            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                            '-file', sdf,
                            '-entity', 'gui_camera',
                            '-timeout', '20',
                        ],
                        capture_output=True, text=True, env=env, timeout=40.0,
                    )
                    out = (result.stdout or '') + (result.stderr or '')
                    if result.returncode == 0:
                        self.log_message.emit(
                            f'已注入 GUI 仿真相机（第 {attempt + 1} 次尝试成功）', 'info'
                        )
                        return
                    # 实体已存在或 service 没好，重试
                    if 'exists' in out.lower():
                        self.log_message.emit('GUI 相机已存在，无需重复注入', 'info')
                        return
                    self.log_message.emit(
                        f'相机注入尝试 {attempt + 1} 失败，重试中...', 'warn'
                    )
                except subprocess.TimeoutExpired:
                    self.log_message.emit('相机注入超时，重试中...', 'warn')
                time.sleep(3.0)
            self.log_message.emit(
                'GUI 相机注入失败，仿真视图将无数据。'
                '确认 Gazebo 在运行后可点「重新生成相机」。', 'warn'
            )

        threading.Thread(target=worker, daemon=True).start()
        return True

    def start_slam(self) -> bool:
        """
        Starting SLAM 建图

        使用 slam_toolbox 的 online_async 模式，适合手动控制建图。

        Returns:
            bool: 启动成功返回 True
        """
        self._stop_process(ProcessType.SLAM)

        self.log_message.emit('正在启动 SLAM 建图 (slam_toolbox)...', 'info')

        env = os.environ.copy()
        env['TURTLEBOT3_MODEL'] = TURTLEBOT3_MODEL

        # 使用 slam_toolbox online_async launch
        cmd = [
            'ros2', 'launch',
            'slam_toolbox',
            'online_async_launch.py',
            'use_sim_time:=true',
        ]

        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=env,
                preexec_fn=os.setsid,
                text=True,
                bufsize=1,
            )
            self._processes[ProcessType.SLAM] = proc
            self.process_started.emit('slam')
            self.log_message.emit(
                'SLAM 建图已启动！请在界面上控制机器人移动来探索环境。', 'info'
            )
            self._start_log_reader(proc, ProcessType.SLAM)
            return True
        except FileNotFoundError:
            self.log_message.emit('启动 SLAM 失败，请检查 slam_toolbox 安装。', 'error')
            return False

    def stop_slam(self):
        """停止 SLAM 建图"""
        self._stop_process(ProcessType.SLAM)
        self.log_message.emit('SLAM 建图已停止', 'info')

    def save_map(self, save_path: str) -> bool:
        """
        异步保存当前地图（不阻塞 GUI 线程）

        使用 nav2_map_server 的 map_saver_cli 保存 SLAM 构建的地图。
        结果通过 map_saved 信号回调通知，避免 subprocess.run 阻塞界面。

        Args:
            save_path: 地图保存路径（不含扩展名），会生成 .pgm 和 .yaml 两个文件

        Returns:
            bool: 保存任务已成功启动返回 True（不代表地图已落盘）
        """
        self.log_message.emit(f'正在保存地图到: {save_path}...', 'info')

        env = os.environ.copy()
        cmd = [
            'ros2', 'run',
            'nav2_map_server',
            'map_saver_cli',
            '-f', save_path,
            '--ros-args', '-p', 'use_sim_time:=true',
        ]

        def worker():
            try:
                result = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    env=env,
                    timeout=30.0,
                )
                if result.returncode == 0:
                    self.log_message.emit(
                        f'地图已保存: {save_path}.pgm, {save_path}.yaml', 'info'
                    )
                    self.map_saved.emit(True, save_path)
                else:
                    self.log_message.emit(f'保存地图失败: {result.stderr}', 'error')
                    self.map_saved.emit(False, save_path)
            except subprocess.TimeoutExpired:
                self.log_message.emit('保存地图超时（30秒），请检查 SLAM 是否正在运行。', 'error')
                self.map_saved.emit(False, save_path)
            except FileNotFoundError:
                self.log_message.emit('未找到 map_saver_cli，请检查 nav2_map_server 安装。', 'error')
                self.map_saved.emit(False, save_path)

        threading.Thread(target=worker, daemon=True).start()
        return True

    def start_navigation(self, map_path: str, planner: str = 'dwa') -> bool:
        """
        Starting Navigation2 导航

        Args:
            map_path: 地图 .yaml 文件的完整路径
            planner: 避障算法，'dwa' / 'teb' / 'mppi'

        Returns:
            bool: 启动成功返回 True
        """
        self._stop_process(ProcessType.NAVIGATION)

        if not os.path.exists(map_path):
            self.log_message.emit(f'地图文件不存在: {map_path}', 'error')
            return False

        # 规划器可用性检查：TEB 在 Humble 无预编译包，没装就别启动坏配置
        if not self.is_planner_available(planner):
            self.log_message.emit(
                f'{planner.upper()} 未安装，已回退到 DWA。'
                f'(TEB 需要 sudo apt install ros-humble-teb-local-planner 或源码编译)',
                'warn'
            )
            planner = 'dwa'

        self.log_message.emit(f'正在启动导航 (算法: {planner.upper()})...', 'info')

        env = os.environ.copy()
        env['TURTLEBOT3_MODEL'] = TURTLEBOT3_MODEL

        # 获取本 package 的 param 文件路径
        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        param_file = os.path.join(pkg_dir, 'robot_gui', 'config', f'{planner}_params.yaml')

        if not os.path.exists(param_file):
            self.log_message.emit(f'参数文件不存在: {param_file}，使用默认 DWA', 'warn')
            param_file = os.path.join(pkg_dir, 'robot_gui', 'config', 'dwa_params.yaml')

        # 使用 nav2_bringup 的 bringup_launch.py 启动导航
        cmd = [
            'ros2', 'launch',
            'nav2_bringup',
            'bringup_launch.py',
            f'map:={map_path}',
            f'params_file:={param_file}',
            'use_sim_time:=true',
        ]

        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=env,
                preexec_fn=os.setsid,
                text=True,
                bufsize=1,
            )
            self._processes[ProcessType.NAVIGATION] = proc
            self.process_started.emit('navigation')
            self.log_message.emit(
                f'导航已启动！地图: {os.path.basename(map_path)}，算法: {planner.upper()}', 'info'
            )
            self._start_log_reader(proc, ProcessType.NAVIGATION)
            return True
        except FileNotFoundError:
            self.log_message.emit('启动导航失败，请检查 nav2_bringup 安装。', 'error')
            return False

    def stop_navigation(self):
        """停止导航"""
        self._stop_process(ProcessType.NAVIGATION)
        self.log_message.emit('导航已停止', 'info')

    def is_planner_available(self, planner: str) -> bool:
        """
        检查某个局部规划器是否已安装可用。

        - dwa / mppi：随 nav2 一起安装，通常总有
        - teb：Humble 无预编译包，需单独安装；这里检测 ros2 包是否存在
        """
        if planner in ('dwa', 'mppi'):
            return True
        if planner == 'teb':
            try:
                result = subprocess.run(
                    ['ros2', 'pkg', 'list'],
                    capture_output=True, text=True, timeout=5.0,
                )
                return result.returncode == 0 and 'teb_local_planner' in result.stdout
            except Exception:
                return False
        return True

    def switch_planner(self, map_path: str, planner: str) -> bool:
        """
        切换避障算法

        本质是停止当前导航，用新的参数重新启动。

        Args:
            map_path: 地图文件路径
            planner: 'dwa' 或 'teb'

        Returns:
            bool: 切换成功返回 True
        """
        self.log_message.emit(f'正在切换到 {planner.upper()} 算法...', 'info')
        return self.start_navigation(map_path, planner)

    def launch_teleop_terminal(self) -> bool:
        """
        在新终端窗口中打开键盘遥控器

        自动检测系统中可用的终端模拟器并Starting turtlebot3_teleop 键盘控制。
        支持的终端：gnome-terminal, xterm, konsole, xfce4-terminal

        Returns:
            bool: 启动成功返回 True
        """
        self.log_message.emit('正在打开键盘遥控终端...', 'info')

        env = os.environ.copy()
        env['TURTLEBOT3_MODEL'] = TURTLEBOT3_MODEL

        teleop_cmd = 'ros2 run turtlebot3_teleop teleop_keyboard'

        # 检测可用的终端模拟器（优先级从高到低）
        terminals = [
            ('gnome-terminal', ['gnome-terminal', '--', 'bash', '-c',
                                f'source /opt/ros/humble/setup.bash && '
                                f'export TURTLEBOT3_MODEL={TURTLEBOT3_MODEL} && '
                                f'{teleop_cmd}; exec bash']),
            ('xfce4-terminal', ['xfce4-terminal', '-e',
                                f'bash -c "source /opt/ros/humble/setup.bash && '
                                f'export TURTLEBOT3_MODEL={TURTLEBOT3_MODEL} && '
                                f'{teleop_cmd}; exec bash"']),
            ('konsole', ['konsole', '-e', 'bash', '-c',
                         f'source /opt/ros/humble/setup.bash && '
                         f'export TURTLEBOT3_MODEL={TURTLEBOT3_MODEL} && '
                         f'{teleop_cmd}; exec bash']),
            ('xterm', ['xterm', '-e',
                       f'bash -c "source /opt/ros/humble/setup.bash && '
                       f'export TURTLEBOT3_MODEL={TURTLEBOT3_MODEL} && '
                       f'{teleop_cmd}; exec bash"']),
        ]

        import shutil
        for term_name, cmd in terminals:
            if shutil.which(term_name):
                try:
                    subprocess.Popen(
                        cmd,
                        env=env,
                        start_new_session=True,  # 脱离父进程
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                    self.log_message.emit(
                        f'已打开键盘遥控器 ({term_name})！\n'
                        f'按键说明: w/s 前进/后退, a/d 左转/右转, q 退出', 'info'
                    )
                    return True
                except Exception as e:
                    self.log_message.emit(f'启动 {term_name} 失败: {e}', 'warn')
                    continue

        self.log_message.emit(
            '未找到可用的终端模拟器！请手动执行:\n'
            f'  ros2 run turtlebot3_teleop teleop_keyboard', 'error'
        )
        return False

    def cleanup_leftovers(self):
        """
        清理上一次运行残留的 ROS2 进程（公开方法）

        应在 GUI 启动时调用一次，确保没有旧进程干扰。
        如果上次程序是异常退出的（如段错误），ROS2 子进程
        可能仍在运行并占用 /map 等话题。
        """
        self._kill_ros_leftovers()
        self.log_message.emit('已清理上一次运行的残留进程', 'info')

    def stop_all(self):
        """停止所有子进程 + 清理残留 ROS2 进程"""
        for proc_type in list(self._processes.keys()):
            self._stop_process(proc_type)
        self._kill_ros_leftovers()
        self.log_message.emit('所有子系统已停止', 'info')

    def is_running(self, proc_type: ProcessType) -> bool:
        """
        检查某个子系统是否正在运行

        Args:
            proc_type: 进程类型

        Returns:
            bool: 运行中返回 True
        """
        proc = self._processes.get(proc_type)
        return proc is not None and proc.poll() is None

    def get_running_processes(self) -> List[str]:
        """
        获取当前运行中的子系统列表

        Returns:
            list: 运行中的进程类型名称列表
        """
        return [
            pt.value for pt, proc in self._processes.items()
            if proc is not None and proc.poll() is None
        ]

    # ===================== 内部方法 =====================

    def _stop_process(self, proc_type: ProcessType):
        """
        安全终止一个子进程及其所有子进程

        先发送 SIGINT（相当于 Ctrl+C），等待后如果未退出则 SIGKILL。

        Args:
            proc_type: 要终止的进程类型
        """
        proc = self._processes.pop(proc_type, None)
        if proc is None:
            return

        if proc.poll() is not None:
            # 进程已经退出
            self.process_stopped.emit(proc_type.value)
            return

        try:
            # 向整个进程组发送 SIGINT
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            # 等待进程退出（最多 5 秒）
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.log_message.emit(
                    f'{proc_type.value} 进程未响应 SIGINT，强制终止...', 'warn'
                )
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=3.0)
        except ProcessLookupError:
            pass  # 进程已经不存在
        except Exception as e:
            self.log_message.emit(f'终止 {proc_type.value} 时出错: {e}', 'error')

        self.process_stopped.emit(proc_type.value)

    def _force_kill_gazebo(self):
        """强制杀掉所有 gzserver / gzclient 残留"""
        self._kill_processes_by_name(['gzserver', 'gzclient'])

    def _kill_ros_leftovers(self):
        """
        清理所有可能残留的 ROS2 进程

        如果程序异常退出（如段错误），ROS2 子进程可能继续运行
        并在后台发布消息（如 /map），导致下次启动时出现数据冲突。
        重启电脑后这些自然消失——所以"重启后 bug 消失"说明清理不彻底。
        """
        targets = [
            'gzserver', 'gzclient',           # Gazebo
            'slam_toolbox',                    # SLAM 建图
            'map_server',                      # 导航地图服务
            'planner_server',                  # 全局规划器
            'controller_server',               # 局部规划器
            'behavior_server',                 # 行为服务器
            'bt_navigator',                    # 行为树导航器
            'amcl',                            # 定位
            'component_container',             # nav2 容器
            'lifecycle_manager',               # 生命周期管理器
        ]
        self._kill_processes_by_name(targets)

    def _kill_processes_by_name(self, names: list):
        """按进程名强制终止"""
        import signal as sig
        for name in names:
            try:
                result = subprocess.run(
                    ['pgrep', '-f', name],
                    capture_output=True, text=True, timeout=2.0
                )
                if result.returncode == 0 and result.stdout.strip():
                    for pid_str in result.stdout.strip().split('\n'):
                        try:
                            pid = int(pid_str)
                            # 不杀自己的进程
                            if pid == os.getpid():
                                continue
                            os.kill(pid, sig.SIGKILL)
                        except (ProcessLookupError, ValueError):
                            pass
            except Exception:
                pass

    def _start_log_reader(self, proc: subprocess.Popen, proc_type: ProcessType):
        """
        启动子进程日志读取

        将子进程的 stdout 输出转化为 log_message 信号，实时显示在 GUI 中。
        将错误和警告信息过滤后转发。

        Args:
            proc: 子进程对象
            proc_type: 进程类型
        """
        # 使用线程读取 stdout（非阻塞）
        import threading

        def reader():
            prefix = proc_type.value.upper()
            try:
                for line in proc.stdout:
                    line = line.strip()
                    if not line:
                        continue
                    # 根据内容判断日志级别
                    if any(kw in line for kw in ['Error', 'ERROR', 'error', 'FATAL']):
                        self.log_message.emit(f'[{prefix}] {line}', 'error')
                    elif any(kw in line for kw in ['Warn', 'WARN', 'warning']):
                        self.log_message.emit(f'[{prefix}] {line}', 'warn')
                    else:
                        # 普通信息，选择性转发（避免刷屏）
                        self.log_message.emit(f'[{prefix}] {line}', 'info')
            except (ValueError, OSError):
                pass  # 进程已关闭

        t = threading.Thread(target=reader, daemon=True)
        t.start()
