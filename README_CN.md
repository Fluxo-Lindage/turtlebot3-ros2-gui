# 🤖 机器人仿真控制平台

基于 **ROS2 Humble + PyQt5** 的 TurtleBot3 仿真控制 GUI 程序。

---

## 📋 功能一览

| 功能 | 说明 |
|------|------|
| 🌍 **仿真环境切换** | 支持标准障碍物场地和室内房屋场景，一键切换 Gazebo 世界 |
| 📊 **实时状态显示** | 实时显示机器人 X/Y 坐标、偏航角、线速度 |
| 🗺️ **SLAM 建图** | 一键启动/停止建图，使用 slam_toolbox |
| 💾 **地图保存** | 建图完成后一键保存为 .pgm/.yaml 地图文件 |
| 🧭 **自主导航** | 加载地图后，设置目标点即可自主导航 |
| ⚙️ **避障算法切换** | 支持 DWA（动态窗口法）和 TEB（时间弹性带）两种算法 |
| 📋 **实时日志** | 显示各子系统的运行日志，方便调试 |

## 🔄 完整工作流程

```
启动仿真 → 开始建图 → 移动机器人探索环境 → 保存地图
    ↓
加载地图 → 选择避障算法 → 开始导航 → 发送目标点 → 机器人自主避障到达
```

---

## 🛠️ 环境要求

| 软件 | 版本/说明 |
|------|-----------|
| Ubuntu | 22.04 |
| ROS2 | Humble |
| Gazebo | (随 ROS2 安装) |
| Python | ≥ 3.8 |
| PyQt5 | ≥ 5.15 |

### 必要的 ROS2 包

```bash
# 基础 TurtleBot3 仿真
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-gazebo

# 导航与建图
sudo apt install ros-humble-turtlebot3-navigation2
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-nav2-map-server

# （可选）TEB 避障算法
sudo apt install ros-humble-teb-local-planner
```

### Python 依赖

```bash
pip3 install PyQt5 --user
```

---

## 🚀 快速启动

### 方法一：一键启动脚本（推荐）

```bash
cd /path/to/Ros_Qt5_Gui_App-master
./start_robot_gui.sh
```

### 方法二：手动启动

```bash
# 1. 加载 ROS2 环境
source /opt/ros/humble/setup.bash

# 2. 设置 TurtleBot3 型号
export TURTLEBOT3_MODEL=waffle_pi

# 3. 启动 GUI
cd /path/to/Ros_Qt5_Gui_App-master
python3 -m robot_gui.main
```

---

## 📖 使用指南

### 第一步：启动仿真环境

1. 在「🌍 仿真环境控制」区域选择场景：
   - **标准障碍物场地** — 带围墙和障碍物的测试场
   - **室内房屋场景** — 模拟家庭环境，有房间和家具
2. 点击 **🚀 启动仿真**，等待 Gazebo 窗口出现

### 第二步：SLAM 建图

1. 点击「🗺️ 建图控制」区的 **🔴 开始建图** 按钮
2. 机器人需要在环境中移动才能构建地图，你可以：
   - 打开新终端，用键盘遥控：`ros2 run turtlebot3_teleop teleop_keyboard`
   - 或使用我们的快速目标功能（在导航启动后）
3. 地图构建满意后，点击 **💾 保存地图**，选择保存位置

### 第三步：自主导航

1. 在「🧭 自主导航控制」区点击 **📂 加载地图**，选择刚才保存的 .yaml 文件
2. 在「⚙️ 避障算法选择」区选择算法：
   - **DWA** — 经典动态窗口法，速度快
   - **TEB** — 轨迹更平滑，适合复杂环境
3. 点击 **▶ 开始导航**，等待导航栈初始化（约 3-5 秒）
4. 在目标点输入框中输入 X、Y 坐标（单位：米），点击 **📍 发送目标点**
5. 机器人将自动规划路径并避障到达目标

### 切换避障算法

- 直接选择 DWA 或 TEB 单选按钮
- 点击「应用算法」即可（如果导航正在运行，会提示自动重启）

---

## 📁 项目结构

```
Ros_Qt5_Gui_App-master/
├── README_CN.md                 # 本文档
├── start_robot_gui.sh           # 一键启动脚本
├── setup.py                     # 包安装配置
├── setup.cfg
├── package.xml                  # ROS2 包清单
│
├── robot_gui/                   # 主 Python 包
│   ├── __init__.py
│   ├── main.py                  # 程序入口，环境检查
│   ├── main_window.py           # PyQt5 主界面（核心）
│   ├── ros_node.py              # ROS2 通信节点
│   ├── process_manager.py       # 子进程管理器
│   └── config/
│       ├── __init__.py
│       ├── dwa_params.yaml      # DWA 算法导航参数
│       └── teb_params.yaml      # TEB 算法导航参数
│
└── src/                         # （原有的 C++ 代码，与本项目无关）
```

---

## 🎓 核心代码说明

### [main_window.py](robot_gui/main_window.py) — 主界面

包含完整的 PyQt5 GUI 布局：
- `MainWindow.__init__()` — 初始化所有 UI 组件和 ROS 节点
- 各 `_create_*_group()` 方法 — 构建各功能分区
- 各 `_on_*` 槽函数 — 处理用户按钮点击

### [ros_node.py](robot_gui/ros_node.py) — ROS2 通信

`RosRobotNode` 类：
- 订阅 `/odom` 获取机器人实时位姿
- 通过 `NavigateToPose` Action 发送导航目标
- 发布 `/initialpose` 用于 AMCL 初始定位

`RosSpinThread` 类：
- 在后台线程中运行 rclpy executor
- 通过 Qt 信号将状态传递到 GUI 线程

### [process_manager.py](robot_gui/process_manager.py) — 进程管理

`ProcessManager` 类：
- 使用 `subprocess.Popen` 启动 Gazebo、SLAM、Navigation
- 支持安全终止（SIGINT → SIGKILL）
- 通过 Qt 信号实时转发子进程输出

---

## ❓ 常见问题

**Q: 启动时提示 "未设置 TURTLEBOT3_MODEL"?**
A: 在终端中执行 `export TURTLEBOT3_MODEL=waffle_pi`，或直接使用 `./start_robot_gui.sh` 启动。

**Q: Gazebo 窗口没有出现?**
A: 首次启动 Gazebo 可能需要较长时间加载模型。如持续失败，请检查：
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Q: 导航启动后机器人不动?**
A: 确保已加载正确的地图文件，然后在程序自动发送初始位姿后（启动 3 秒后），再发送导航目标。

**Q: TEB 算法不可用?**
A: TEB 需要额外安装：`sudo apt install ros-humble-teb-local-planner`

**Q: 保存地图时提示超时?**
A: 确保 SLAM 正在运行且机器人已经探索了部分环境，至少需要有一些地图数据才能保存。

---

## 📝 课程设计报告要点

这个项目涵盖了以下知识点：

1. **ROS2 通信机制** — 话题订阅/发布、Action 客户端
2. **PyQt5 GUI 编程** — 信号/槽机制、多线程
3. **SLAM 原理** — 基于 slam_toolbox 的实时建图
4. **路径规划** — 全局规划（Navfn）+ 局部规划（DWA/TEB）
5. **自主导航** — AMCL 定位 + 代价地图 + 行为树
6. **避障算法** — DWA（速度空间采样）vs TEB（轨迹优化）
7. **进程管理** — subprocess 管理 ROS2 子进程

---

## 📄 License

Apache License 2.0

---

*祝课程设计顺利通过！🎉*
