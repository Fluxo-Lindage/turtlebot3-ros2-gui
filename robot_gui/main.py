#!/usr/bin/env python3
"""
机器人仿真控制平台 — 主入口

基于 ROS2 Humble + PyQt5 的 TurtleBot3 仿真控制 GUI。

用法:
    python3 main.py

前置条件:
    1. 已安装 ROS2 Humble
    2. 已安装 turtlebot3, turtlebot3_gazebo, navigation2, slam_toolbox
    3. 已 source ROS2 环境: source /opt/ros/humble/setup.bash
    4. 已设置 TURTLEBOT3_MODEL 环境变量: export TURTLEBOT3_MODEL=waffle_pi

功能:
    - 选择和启动不同的 Gazebo 仿真环境
    - 实时显示机器人位置和速度
    - 一键 SLAM 建图、保存地图
    - 加载地图并启动自主导航
    - 在 DWA 和 TEB 避障算法之间切换
"""

import sys
import os


def check_environment() -> bool:
    """
    检查必要的环境变量和 ROS2 包是否就绪

    Returns:
        bool: 环境就绪返回 True，否则返回 False
    """
    issues = []

    # 检查 TURTLEBOT3_MODEL
    model = os.environ.get('TURTLEBOT3_MODEL', '')
    if not model:
        issues.append('[失败] 未设置 TURTLEBOT3_MODEL 环境变量\n'
                      '   请在终端中执行: export TURTLEBOT3_MODEL=waffle_pi')
    else:
        print(f'[OK] TURTLEBOT3_MODEL = {model}')

    # 检查 ROS 2 环境
    ros_distro = os.environ.get('ROS_DISTRO', '')
    if not ros_distro:
        issues.append('[失败] 未找到 ROS 2 环境\n'
                      '   请在终端中执行: source /opt/ros/humble/setup.bash')
    else:
        print(f'[OK] ROS_DISTRO = {ros_distro}')

    # 检查 PyQt5
    try:
        from PyQt5 import QtCore
        print(f'[OK] PyQt5 版本: {QtCore.PYQT_VERSION_STR}')
    except ImportError:
        issues.append('[失败] 未安装 PyQt5\n'
                      '   请执行: pip3 install PyQt5')

    # 检查 rclpy
    try:
        import rclpy
        print('[OK] rclpy 可用')
    except ImportError:
        issues.append('[失败] 未安装 rclpy\n'
                      '   请确保已正确安装 ROS2 Humble')

    if issues:
        print('\n' + '=' * 60)
        print('环境检查失败，请先解决以下问题：\n')
        for issue in issues:
            print(issue)
            print()
        print('=' * 60)
        return False

    print('[OK] 环境检查通过！')
    return True


def main():
    """程序主入口"""
    print('机器人仿真控制平台 v1.0.0')
    print('=' * 60)

    # 环境检查
    if not check_environment():
        sys.exit(1)

    # 确保地图目录存在
    map_dir = os.path.expanduser('~/robot_maps')
    os.makedirs(map_dir, exist_ok=True)

    # 创建 Qt 应用
    from PyQt5.QtWidgets import QApplication

    app = QApplication(sys.argv)
    app.setApplicationName('Robot Control Platform')
    app.setOrganizationName('RobotLab')

    # 设置全局默认字体（适配高分辨率显示器）
    from PyQt5.QtGui import QFont
    default_font = QFont('Sans', 11)
    app.setFont(default_font)

    # 设置应用图标（如需要）
    # app.setWindowIcon(QIcon('path/to/icon.png'))

    # 创建并显示主窗口
    from robot_gui.main_window import MainWindow
    window = MainWindow()
    window.show()

    # 启动 Qt 事件循环
    exit_code = app.exec_()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
