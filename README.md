# 机器人仿真控制平台 — 从零开始学习指南

> 基于 **ROS2 Humble + PyQt5** 的 TurtleBot3 仿真控制 GUI 程序
>
> 适用人群：ROS 完全 0 基础、刚接触机器人学的同学
>
> 预计阅读时间：30-45 分钟

---

## 目录

1. [准备工作：你需要了解的 5 个核心概念](#1-准备工作你需要了解的-5-个核心概念)
2. [这个项目做了什么（一句话版）](#2-这个项目做了什么一句话版)
3. [系统架构全景图](#3-系统架构全景图)
4. [操作流程：从启动到自主导航](#4-操作流程从启动到自主导航)
5. [模块详解：每个 Python 文件做了什么](#5-模块详解每个-python-文件做了什么)
6. [配置文件详解](#6-配置文件详解)
7. [代码导读：关键代码逐段讲解](#7-代码导读关键代码逐段讲解)
8. [常见问题排查](#8-常见问题排查)
9. [课程设计报告素材](#9-课程设计报告素材)

---

## 1. 准备工作：你需要了解的 5 个核心概念

如果你对 ROS 完全不了解，请先花 10 分钟读懂这 5 个概念。它们贯穿整个项目。

### 概念一：ROS2 是什么？

**ROS2**（Robot Operating System 2）并不是一个真正的操作系统，而是一个**分布式通信框架**。

想象你有一台机器人，上面跑着几十个程序：一个程序读激光雷达数据，一个程序算路径，一个程序控制电机……这些程序之间需要互相通信。ROS2 就提供了一套标准化的通信机制，让这些程序能够互相"对话"。

```
[程序A: 激光雷达] ----发布数据----> [ROS2 通信总线] ----订阅数据----> [程序B: 避障]
```

本项目中，我们用到的 ROS2 通信方式有三种：

| 通信方式 | 比喻 | 本项目中的用途 |
|----------|------|---------------|
| **Topic（话题）** | 广播电台。一个节点发布，多个节点可以收听 | 订阅 `/odom`（里程计数据）、`/map`（地图数据） |
| **Action（动作）** | 外卖下单。客户端发请求，服务端执行并反馈进度，最后告知结果 | 发送导航目标点 `NavigateToPose` |
| **TF（坐标变换）** | GPS 定位系统中的坐标系转换 | 把机器人在 odom 坐标系的位置转换到 map 坐标系 |

### 概念二：Gazebo 仿真环境

**Gazebo** 是一个 3D 物理仿真器。如果拿游戏打比方，它相当于机器人的"游戏引擎"——它模拟了一个有物理规律的世界，里面有地面、墙壁、障碍物，还有一台 TurtleBot3 机器人。

当你在 Gazebo 中启动仿真时：
- Gazebo "画"出一个世界（world），里面有墙壁和障碍物
- TurtleBot3 机器人模型被加载到这个世界中
- 机器人上虚拟的激光雷达（LiDAR）开始"发射"激光束，探测周围环境
- 虚拟的电机接收速度指令并驱动机器人移动

最重要的是：**Gazebo 对外暴露的接口和真实机器人完全相同**。你在仿真中写的代码，可以直接部署到真实机器人上运行。

### 概念三：SLAM — 同时定位与建图

**SLAM** = Simultaneous Localization And Mapping（同时定位与构建地图）

这是机器人学最核心的问题之一。想象你被蒙着眼睛放进一个陌生房间：

1. 你摸到一面墙 — 你知道"这里有一面墙"
2. 你往前走两步 — 你知道"我大概走了两米"
3. 你又摸到一张桌子 — 你知道"墙旁边有张桌子"
4. 同时你也在更新自己对"我在哪里"的估计

机器人做 SLAM 也是类似的过程。本项目使用 **slam_toolbox** 这个 ROS2 包来实现 SLAM：
- 传感器输入：激光雷达扫描数据（`/scan`）+ 里程计（`/odom`）
- 输出：一张占据栅格地图（OccupancyGrid），标记了哪里是空闲、哪里有障碍

### 概念四：导航栈 — 机器人如何自主走到目的地

ROS2 的 Navigation2（简称 Nav2）是一个完整的导航框架。当你给机器人一个目标点，导航栈内部做了这些事情：

```
你点击地图设置目标
        |
        v
[AMCL 定位] ------> "我目前知道自己在地图上的位置"
        |
        v
[全局规划器] ------> "在地图上找到一条从当前位置到目标的路径"（大方向）
        |
        v
[局部规划器] ------> "根据当前传感器数据，实时避开障碍物"（每0.05秒更新）
        |
        v
[速度指令] --------> 发给机器人底盘 "/cmd_vel"
```

关键名词解释：

| 名词 | 通俗解释 |
|------|----------|
| **AMCL** | 自适应蒙特卡洛定位。用粒子滤波算法估算机器人在地图上的位置 |
| **全局规划器（Global Planner）** | 看大图，规划从 A 到 B 的最短路径（本项目用 Navfn） |
| **局部规划器（Local Planner）** | 看眼前，实时避障（本项目用 DWA 或 TEB） |
| **代价地图（Costmap）** | 一张"热量图"，越靠近障碍物得分越高，机器人会避开高分区域 |
| **行为树（Behavior Tree）** | 机器人做决策的流程图（如：先规划→规划失败就原地旋转→还不行就后退……） |

### 概念五：DWA vs TEB — 两种避障算法

两者都是局部规划器，区别如下：

| | **DWA（动态窗口法）** | **TEB（时间弹性带）** |
|---|---|---|
| 思路 | 在速度空间中采样，选最优速度指令 | 优化整条轨迹的形状和时间分配 |
| 计算量 | 小，实时性好 | 较大，但轨迹更平滑 |
| 适用场景 | 简单环境、狭窄通道 | 复杂环境、需要平滑转弯 |
| ROS2 包 | `dwb_core`（内置） | `teb_local_planner`（需额外安装） |
| 轨迹质量 | 有时会走锯齿状 | 非常平滑，像汽车转弯 |

---

## 2. 这个项目做了什么（一句话版）

> 这是一个**带图形界面的 ROS2 机器人仿真控制台**。你把所有需要用命令行敲的 ros2 launch 指令变成了按钮。点一下就能启动 Gazebo 仿真、SLAM 建图、自主导航，并且地图和机器人状态实时显示在界面上。

说人话就是：**像一个机器人版的"遥控器 App"，只不过控制的是仿真里的 TurtleBot3 机器人**。

---

## 3. 系统架构全景图

```
+------------------------------------------------------------------+
|                        PyQt5 GUI 主界面                            |
|  +-----------+ +--------------------+ +--------------------+      |
|  | 控制面板   | |   实时地图显示      | |  机器人状态面板      |      |
|  |           | |                    | |  位置 X/Y/朝向      |      |
|  | [启动仿真] | |  +----------+      | |  速度 线/角         |      |
|  | [开始建图] | |  |          |      | |  里程计/算法/地图   |      |
|  | [保存地图] | |  | Occupancy|      | |                    |      |
|  | [加载地图] | |  |  Grid    |      | |  操作提示           |      |
|  | [开始导航] | |  |  地图    |      | |                    |      |
|  | [发送目标] | |  |          |      | +--------------------+      |
|  |           | |  +----------+      |                             |
|  | DWA/TEB   | |  机器人图标+轨迹    |                             |
|  | 算法切换   | |  导航目标旗帜       |                             |
|  +-----------+ +--------------------+                             |
|  |                 系统日志面板                                   |
|  +---------------------------------------------------------------+      |
+------------------------------------------------------------------+
        |                  |                  |
        | Qt Signals       |                  | Qt Signals
        v                  v                  v
+---------------+  +-------------+  +------------------+
| ProcessManager|  | MapWidget   |  |  RosRobotNode     |
| (subprocess)  |  | (QPainter)  |  |  (rclpy Node)     |
|               |  |             |  |                   |
| 启动/停止:    |  | 渲染地图    |  | 订阅 /odom        |
|  - Gazebo     |  | 渲染机器人  |  | 订阅 /map         |
|  - SLAM       |  | 渲染目标    |  | 发布 /initialpose |
|  - Navigation |  | 鼠标交互    |  | Action: 导航目标  |
+------+--------+  +-------------+  | TF: odom→map      |
       |                            +--------+----------+
       | subprocess.Popen                    | rclpy
       v                                     v
+--------------------------------------------------+
|                  ROS2 网络层                       |
|  /odom  /scan  /map  /cmd_vel  /initialpose       |
|  navigate_to_pose (Action)                        |
+--------------------------------------------------+
       |
       v
+--------------------------------------------------+
|            Gazebo 仿真器 + TurtleBot3              |
|  物理引擎 / 传感器模拟 / 3D 渲染                    |
+--------------------------------------------------+
```

数据流说明：

1. **Gazebo** 仿真出机器人在物理世界中的运动，产生传感器数据（激光雷达、里程计等），发布到 ROS2 话题
2. **RosRobotNode** 订阅这些话题，获取机器人实时位置和地图数据
3. 通过 **Qt 信号**机制，ROS 线程将数据安全地传递到 GUI 主线程
4. **MapWidget** 用 QPainter 将地图数据渲染成图像，同时画出机器人和目标点
5. 用户点击按钮，**ProcessManager** 用 subprocess 启动/停止对应的 ROS2 launch 进程
6. 用户在地图上点击目标，**RosRobotNode** 通过 Action 客户端发送到导航栈

---

## 4. 操作流程：从启动到自主导航

### 完整操作步骤（初次使用跟着做一遍）

#### 第一步：启动程序

```bash
cd /home/robot/07091430bugdefuse
./start_robot_gui.sh
```

启动脚本会自动检查 ROS2 环境、设置 TurtleBot3 型号、检查依赖。

#### 第二步：启动仿真环境

1. 在左侧"仿真环境"区域，选择场景（推荐先选"标准障碍物场地"）
2. 点击 **启动仿真**
3. 等待 Gazebo 窗口出现（首次启动可能需要 10-30 秒加载模型）
4. 顶部状态栏 **Gazebo** 文字变绿，表示仿真运行中

#### 第三步：打开键盘遥控器（方便手动建图）

点击 **打开键盘遥控器**，会弹出一个新终端窗口。
- `w/s`：前进/后退
- `a/d`：左转/右转
- `q`：退出

#### 第四步：SLAM 建图

1. 确认 Gazebo 正在运行（顶部 Gazebo 指示灯绿色）
2. 点击 **开始建图**
3. 顶部状态栏 **SLAM** 文字变绿
4. 用键盘遥控器驱动小车在环境中四处走动
5. 观察中间地图区域，灰色区域逐渐被填充为米白色（空闲区域）和深色（障碍物）
6. 当你对地图满意后，点击 **保存地图**

地图文件会生成两个文件：
- `map_20240709_143025.pgm` — 图片格式的地图
- `map_20240709_143025.yaml` — 地图的描述文件（分辨率、原点等信息）

#### 第五步：自主导航

1. 点击 **开始导航**（如果 SLAM 还在运行，程序会自动停止它）
2. 顶部状态栏 **导航** 文字变绿
3. 等待 3-5 秒让 AMCL 完成初始定位
4. 通过以下任一方式发送目标点：
   - **点击地图**：直接在地图上点一下（最直观）
   - **手动输入坐标**：在 X/Y 输入框中填写坐标，点击"发送"
   - **预设按钮**：点击"原点"、"(2,0)"等快速目标
5. 观察机器人自动规划路径并移动

#### 切换避障算法

在"避障算法"区域选择 DWA 或 TEB，点击 **应用算法**。如果导航正在运行，会提示是否重启。

#### 停止一切

点击红色的 **一键停止所有** 按钮，所有子系统（Gazebo、SLAM、导航）全部停止。

---

## 5. 模块详解：每个 Python 文件做了什么

```
robot_gui/
├── __init__.py           # 让 Python 把这个目录识别为一个"包"
├── main.py               # 程序入口：环境检查 + 启动 GUI
├── main_window.py        # 核心！PyQt5 主界面全部在这里
├── ros_node.py           # ROS2 通信层：订阅/发布/Action/TF
├── process_manager.py    # 子进程管理：启动/停止 Gazebo/SLAM/导航
├── map_widget.py         # 自定义地图控件：渲染占据栅格+交互
└── config/
    ├── __init__.py
    ├── dwa_params.yaml   # DWA 算法的导航参数
    └── teb_params.yaml   # TEB 算法的导航参数
```

### [main.py](robot_gui/main.py) — 程序入口

**你不需要读懂全部代码**，只需知道它做了 4 件事：

1. `check_environment()`：启动前检查 ROS2 环境是否就绪、PyQt5 是否安装、rclpy 是否可用
2. 创建 Qt 应用程序 `QApplication`
3. 创建主窗口 `MainWindow` 并显示
4. 进入 Qt 事件循环 `app.exec_()`（程序就在这里一直运行，直到你关掉窗口）

关键代码：

```python
# 这段是程序运行的核心流程
app = QApplication(sys.argv)          # 创建 Qt 应用
window = MainWindow()                 # 创建主窗口
window.show()                         # 显示窗口
exit_code = app.exec_()               # 进入事件循环（阻塞在这里直到窗口关闭）
```

### [main_window.py](robot_gui/main_window.py) — 主界面（最核心的模块）

这是整个项目最核心的文件，约 500 行。它负责：

**布局结构（`_init_ui` 方法）：**

```
+---------------------------------------------------------------+
|  顶部状态栏（_create_top_bar）                                   |
|  标题 + Gazebo 指示灯 + SLAM 指示灯 + 导航指示灯 + 机器人指示灯     |
+---------------------------------------------------------------+
| 左侧            | 中间                 | 右侧                    |
| 控制面板         | 地图控件              | 机器人状态面板           |
| (_create_       | (MapWidget)          | (_create_robot_        |
|  control_panel) |                      |  state_panel)          |
|                 |                      |                        |
| 仿真环境选择     | 占据栅格地图           | 位置: X/Y/朝向          |
| SLAM 建图控制   | 机器人图标             | 速度: 线速度/角速度      |
| 自主导航控制     | 导航目标标记           | 状态: 里程计/算法/地图   |
| 避障算法选择     | 轨迹线                 |                        |
| 一键停止         |                      |                        |
+---------------------------------------------------------------+
| 底部                                                           |
| 系统日志面板（_create_log_panel）                                |
+---------------------------------------------------------------+
```

**核心逻辑流程（以"启动导航"为例）：**

```
用户点击"开始导航"
    |
    v
_on_start_nav() 被调用
    |
    +-- 检查有没有加载地图？（没有 → 弹窗提醒）
    +-- 检查仿真在运行吗？（没有 → 弹窗提醒）
    +-- SLAM 还在运行吗？（在 → 自动停止，因为它会和导航冲突）
    +-- 读取当前选择的算法（DWA 还是 TEB）
    +-- process_manager.start_navigation(map_path, planner)
    +-- 5秒后自动发送 AMCL 初始位姿（因为刚启动时定位需要初始值）
    +-- 启用"发送目标"和"取消目标"按钮
```

**状态指示灯逻辑（`_update_status_indicators` 方法）：**

每 500 毫秒检查一次：
- Gazebo 进程是否还在运行？
- SLAM 进程是否还在运行？
- 导航进程是否还在运行？
- 机器人里程计有新数据吗？

对应的顶部标签：
- **绿色** 文字 = 正常运行
- **红色** 文字 = 未运行/异常

这就是界面顶部那四个红绿灯提示的实现原理。

### [ros_node.py](robot_gui/ros_node.py) — ROS2 通信节点

这是 GUI 和 ROS2 之间的"翻译官"。它包含两个类：

**`RosRobotNode(Node)`** — ROS2 节点：

```python
# 订阅（从 ROS2 接收数据）
self.odom_sub = self.create_subscription(Odometry, '/odom', ...)  # 订阅里程计
self.map_sub = self.create_subscription(OccupancyGrid, '/map', ...)  # 订阅地图

# 发布（向 ROS2 发送数据）
self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', ...)

# Action 客户端（发送带反馈的请求）
self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

# TF 坐标变换
self._tf_buffer = Buffer()  # 缓存坐标系之间的变换关系
self._tf_listener = TransformListener(self._tf_buffer, self)  # 监听变换更新
```

核心方法：

| 方法 | 功能 |
|------|------|
| `get_robot_state()` | 获取机器人当前状态（通过 TF 从 odom 转到 map 坐标） |
| `get_map_data()` | 获取最新地图数据（已翻转使行序与 QImage 一致） |
| `send_navigation_goal(x, y, yaw)` | 发送导航目标点 |
| `cancel_navigation()` | 取消当前导航任务 |
| `publish_initial_pose(x, y, yaw)` | 发布初始位姿给 AMCL 定位 |

**`RosSpinThread(QThread)`** — 后台运行线程：

```
主线程（GUI 更新界面）          后台线程（ROS2 通信）
        |                            |
        |                            | while running:
        |                            |   spin_once() — 处理 ROS2 消息
        |                            |   每 100ms: get_robot_state() → 通过信号发给 GUI
        |                            |   每 500ms: get_map_data() → 通过信号发给 GUI
        |                            |
        | <--- state_updated 信号 --- |
        | <--- map_updated 信号 ------ |
        |                            |
更新左侧状态面板                |
更新地图显示                    |
```

为什么需要两个线程？因为 ROS2 的 `spin_once()` 会阻塞，如果在主线程中调用，GUI 就会卡顿。把 ROS2 通信放到单独的线程中，GUI 始终保持流畅。

### [process_manager.py](robot_gui/process_manager.py) — 进程管理器

这个模块负责启动和停止所有的 ROS2 子进程。

**为什么需要它？** 因为 ROS2 的 Gazebo、SLAM、导航每个都是独立的程序（通过 `ros2 launch` 启动），它们不能直接在我们的 Python 进程中运行。我们需要用 `subprocess.Popen` 来启动它们，并管理它们的生命周期。

核心方法：

| 方法 | 启动的 ROS2 命令 |
|------|-----------------|
| `start_gazebo(world_key)` | `ros2 launch turtlebot3_gazebo <world>.launch.py` |
| `start_slam()` | `ros2 launch slam_toolbox online_async_launch.py` |
| `start_navigation(map_path, planner)` | `ros2 launch nav2_bringup bringup_launch.py map:=<path>` |
| `save_map(path)` | `ros2 run nav2_map_server map_saver_cli -f <path>` |
| `launch_teleop_terminal()` | 在新终端中运行 `ros2 run turtlebot3_teleop teleop_keyboard` |

**进程终止的安全设计：**

```
_stop_process(proc_type):
    1. 发送 SIGINT（相当于 Ctrl+C），等待 5 秒
    2. 如果进程还在运行 → 发送 SIGKILL（强制杀死）
    3. 如果进程已不存在 → 忽略
```

**残留进程清理（`_kill_ros_leftovers`）：**

如果程序异常退出（比如你强制关掉了窗口），ROS2 子进程可能变成"孤儿进程"继续在后台运行。这会导致下一次启动时出现数据冲突。所以程序启动时会先调用 `cleanup_leftovers()` 清理：
- gzserver / gzclient（Gazebo 相关）
- slam_toolbox
- map_server / planner_server / controller_server / behavior_server / bt_navigator / amcl（导航相关）

### [map_widget.py](robot_gui/map_widget.py) — 实时地图可视化

这是一个纯 Qt 绘图组件，不依赖任何 ROS2 库。它用 `QPainter` 在控件上画图。

**核心工作流程：**

```
ROS 线程收到 /map 话题数据
        |
        v
update_map() 被调用
  数据存储在 self._map_data (numpy 数组)
  标记 self._map_dirty = True
  调用 self.update() → 触发 paintEvent()
        |
        v
paintEvent() 被调用:
  1. _paint_map()          # 绘制占据栅格地图
  2. _paint_grid()         # 绘制坐标网格（可选）
  3. _paint_robot_trail()  # 绘制机器人运动轨迹
  4. _paint_goal()         # 绘制导航目标点（如果有）
  5. _paint_robot()        # 绘制机器人当前位置 + 方向箭头
```

**占据栅格地图像素映射：**

```python
# OccupancyGrid 中每个格子的值含义：
if val < 0:   # -1: 未知区域 → 暖灰色 (210, 200, 180)
elif val == 0: # 0: 空闲区域 → 米白色 (252, 247, 235)
else:           # 1-100: 障碍物 → 从浅棕过渡到深棕
```

**坐标转换（世界坐标 ↔ 屏幕像素）：**

```python
# 世界坐标 → 屏幕像素
ix = (world_x - map_origin_x) / resolution        # 世界 → 图像
iy = map_height - (world_y - map_origin_y) / resolution  # 翻转 Y 轴
sx = center_x + (ix - map_width/2) * zoom + offset_x     # 图像 → 屏幕
sy = center_y + (iy - map_height/2) * zoom + offset_y
```

**鼠标交互：**

| 操作 | 效果 |
|------|------|
| 左键点击 | 设置导航目标点 |
| 右键点击 | 清除导航目标 |
| 滚轮滚动 | 放大/缩小地图 |
| 中键拖动 / Ctrl+左键拖动 | 平移地图 |
| 双击左键 | 自动缩放适应窗口 |
| R 键 | 重置视角 |
| F 键 | 适应窗口 |
| G 键 | 切换网格显示 |
| C 键 | 清除运动轨迹 |
| Esc | 清除目标点 |

---

## 6. 配置文件详解

### DWA 参数 ([dwa_params.yaml](robot_gui/config/dwa_params.yaml))

这个文件配置了整个 Nav2 导航栈的行为。最关键的参数：

**机器人物理参数：**

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      max_vel_x: 0.22        # 最大前进速度 0.22 m/s
      max_vel_theta: 1.0     # 最大旋转速度 1.0 rad/s (约 57°/s)
      acc_lim_x: 2.5         # 线加速度限制
      acc_lim_theta: 3.2     # 角加速度限制
      vx_samples: 20         # 速度采样点数（越大越精确但也越慢）
      vtheta_samples: 40     # 角速度采样点数
      sim_time: 2.0          # 向前仿真2秒来评估轨迹
```

**代价地图参数：**

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      width: 3              # 局部代价地图 3m x 3m（以机器人为中心）
      height: 3
      resolution: 0.05      # 每个格子 5cm
      robot_radius: 0.22    # 机器人半径 22cm（用于碰撞检测）
      inflation_radius: 1.0 # 障碍物膨胀半径 1m（安全距离）
```

**目标容差：**

```yaml
general_goal_checker:
  xy_goal_tolerance: 0.25  # 到达目标 25cm 以内就算成功
  yaw_goal_tolerance: 0.25 # 朝向偏差 0.25 rad（约14°）以内就算成功
```

### TEB 参数 ([teb_params.yaml](robot_gui/config/teb_params.yaml))

TEB 与 DWA 共享大部分结构（AMCL、代价地图、规划器等都一样），只在局部规划器部分不同：

```yaml
FollowPath:
  plugin: "teb_local_planner::TEBLocalPlanner"  # 使用 TEB 而不是 DWB
  dt_ref: 0.3                  # 轨迹点间时间间隔 0.3s
  min_obstacle_dist: 0.22      # 最小障碍物距离
  inflation_dist: 0.6          # 障碍物膨胀距离
  no_inner_iterations: 5       # 内循环优化迭代次数
  no_outer_iterations: 4       # 外循环优化迭代次数
  weight_obstacle: 50.0        # 避障权重
  weight_inflation: 100.0      # 膨胀层权重
  weight_optimaltime: 1.0      # 时间最优权重
  enable_homotopy_class_planning: True  # 开启同伦类规划（可走不同拓扑路径）
```

---

## 7. 代码导读：关键代码逐段讲解

### 7.1 Qt 信号/槽机制（GUI 线程安全的核心）

ROS2 数据在后台线程中接收，但 Qt 规定 **GUI 只能在主线程中更新**。Qt 的解决方案是"信号/槽"：

```python
# ros_node.py — 后台线程中
class RosSpinThread(QThread):
    state_updated = pyqtSignal(dict)  # 定义信号

    def run(self):
        while self._running:
            state = self._ros_node.get_robot_state()
            self.state_updated.emit(state)  # 发送信号（线程安全）

# main_window.py — 主线程中
class MainWindow(QMainWindow):
    def _connect_signals(self):
        # 连接信号到槽函数
        self._ros_thread.state_updated.connect(self._on_state_updated)

    @pyqtSlot(dict)
    def _on_state_updated(self, s):
        # 这个函数在主线程中执行，可以安全地更新 GUI
        self._lbl_pos_x.setText(f'{s["x"]:.3f} m')
```

### 7.2 OccupancyGrid 地图数据翻转（重要细节）

ROS2 的 OccupancyGrid 格式中，`row 0` = 地图的底部（y 最小值处），但 QImage 中 `row 0` = 图片的顶部。如果不翻转，画出来的地图就是上下颠倒的！

```python
# ros_node.py 中的修复
def _map_callback(self, msg: OccupancyGrid):
    data = np.array(msg.data, dtype=np.int8).reshape(h, w)
    data = np.flipud(data)  # 翻转！使 row 0 = 地图顶部，与 QImage 一致
```

### 7.3 TF 坐标变换（机器人真实位置的计算）

机器人里程计发布的位置在 `odom` 坐标系中，但导航需要的是 `map` 坐标系。AMCL 会维护从 `map` 到 `odom` 的变换。我们通过 TF 查询来获得 `map` 坐标系下的位置：

```python
def get_robot_state(self):
    # 查询 map → base_link 的变换
    trans = self._tf_buffer.lookup_transform('map', 'base_link', RosTime())
    map_x = trans.transform.translation.x
    map_y = trans.transform.translation.y
    # 从四元数计算偏航角
    map_yaw = self._quaternion_to_yaw(q.x, q.y, q.z, q.w)
```

如果 TF 还不可用（比如导航刚启动、AMCL 还没收敛），就回退到使用 odom 坐标。

### 7.4 子进程安全终止设计

```python
def _stop_process(self, proc_type: ProcessType):
    proc = self._processes.pop(proc_type, None)
    # 1. 先发送 SIGINT（Ctrl+C），给进程机会自己清理
    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    try:
        proc.wait(timeout=5.0)  # 等 5 秒
    except subprocess.TimeoutExpired:
        # 2. 5秒后还没退出 → 直接 SIGKILL 强制杀死
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        proc.wait(timeout=3.0)
```

为什么用 `os.killpg()` 而不是 `proc.kill()`？因为 `ros2 launch` 会创建一整棵进程树（父进程 + 多个子进程），`killpg` 向整个进程组发信号，确保一个不留。

### 7.5 地图渲染性能优化

如果每次都把整个 numpy 数组转成 QImage，在 CPU 上逐像素 setPixel，当收到高频地图更新时会很卡。解决方案：

```python
# 1. 缓存机制 — 只在数据变化时重建
if self._map_dirty:
    self._rebuild_map_pixmap()   # 重建 QPixmap 缓存
    self._map_dirty = False
painter.drawPixmap(target_rect, self._map_pixmap)  # 直接绘制缓存

# 2. 防抖 — 高频更新时跳过中间帧
def _on_map_updated(self, m):
    self._last_map_render_time = time.time()
    QTimer.singleShot(300, lambda: self._render_map_if_stable(m, now))
    # 只在 300ms 内没有新地图数据时才实际渲染
```

---

## 8. 常见问题排查

### 启动类问题

**Q: `./start_robot_gui.sh` 提示 "未找到 ROS2 Humble 环境"？**

A: ROS2 没有正确安装或安装路径不对。检查 `/opt/ros/humble/setup.bash` 是否存在：
```bash
ls /opt/ros/humble/setup.bash
```
如果不存在，需要先安装 ROS2 Humble。

**Q: 报错 "未设置 TURTLEBOT3_MODEL"？**

A: 用启动脚本启动即可，脚本会自动设置。如果手动启动需要先：
```bash
export TURTLEBOT3_MODEL=waffle_pi
```

**Q: 报错 "ImportError: No module named PyQt5"？**

A: PyQt5 没安装（或者装到了错误的 Python 版本）：
```bash
pip3 install PyQt5 --user
```

### 运行类问题

**Q: Gazebo 窗口没有出现/闪退？**

A: 可能原因：
1. 显卡驱动问题：尝试 `export LIBGL_ALWAYS_SOFTWARE=1` 然后用软件渲染
2. 首次启动加载模型慢：耐心等 30 秒以上
3. 内存不够：Gazebo 至少需要 2GB 空闲内存
4. 手动测试 Gazebo 能否独立运行：
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

**Q: 点击"开始建图"后地图区域一直是空白的？**

A: SLAM 需要机器人移动才能建图。用键盘遥控器驱动小车走一圈，地图就会逐渐出现。

**Q: 保存地图时提示超时？**

A: 确保：
1. SLAM 确实在运行（顶部 SLAM 指示灯绿色）
2. 已经移动了机器人，有了一定的地图数据
3. 地图保存路径有写入权限

**Q: 导航启动后机器人不动？**

A: 检查步骤：
1. 是否加载了正确的地图文件（.yaml）？
2. 等待 5 秒让 AMCL 完成初始定位
3. 尝试发送一个离当前位置近一些的目标点
4. 查看系统日志面板，看有没有报错

**Q: TEB 算法点了没反应？**

A: TEB 需要编译安装（ROS2 Humble 没有预编译包）：
```bash
sudo apt install ros-humble-teb-local-planner
```
如果没有这个包，先使用 DWA（已内置，无需额外安装）。

**Q: 第二次启动程序时，地图显示异常/机器人位置乱跳？**

A: 上次程序异常退出留下的残留进程在搞鬼。程序启动时会自动调用 `cleanup_leftovers()` 清理。如果还不行，手动清理：
```bash
pkill -9 gzserver; pkill -9 gzclient; pkill -9 slam_toolbox
pkill -9 planner_server; pkill -9 controller_server
pkill -9 map_server; pkill -9 amcl; pkill -9 bt_navigator
```

---

## 9. 课程设计报告素材

这个项目涉及了机器人学和软件工程的多个知识点，你可以在报告中提到：

### 涉及的知识领域

| 领域 | 具体内容 | 对应代码 |
|------|---------|---------|
| **ROS2 通信机制** | Topic 订阅/发布、Action 客户端、TF 坐标变换 | [ros_node.py](robot_gui/ros_node.py) |
| **PyQt5 GUI 编程** | 主窗口布局、信号/槽机制、自定义 Widget（QPainter 绘图）、多线程 | [main_window.py](robot_gui/main_window.py), [map_widget.py](robot_gui/map_widget.py) |
| **SLAM 原理** | 基于 slam_toolbox 的 online_async 实时建图 | [process_manager.py](robot_gui/process_manager.py) |
| **路径规划** | 全局规划（Navfn）+ 局部规划（DWA/TEB） | [dwa_params.yaml](robot_gui/config/dwa_params.yaml), [teb_params.yaml](robot_gui/config/teb_params.yaml) |
| **自主导航** | AMCL 自适应蒙特卡洛定位 + 代价地图 + 行为树 | 同上 |
| **避障算法对比** | DWA（速度空间采样）vs TEB（轨迹优化） | 同上 |
| **进程管理** | subprocess.Popen 管理外部程序、进程组信号控制 | [process_manager.py](robot_gui/process_manager.py) |
| **坐标变换与渲染** | 世界坐标↔图像坐标↔屏幕坐标、Y轴翻转 | [map_widget.py](robot_gui/map_widget.py) |
| **线程安全** | Qt 线程安全数据传递、numpy 数据锁保护 | [ros_node.py](robot_gui/ros_node.py), [main_window.py](robot_gui/main_window.py) |

### 技术亮点

1. **一键式操作**：把复杂的 `ros2 launch` 命令封装为 GUI 按钮，降低使用门槛
2. **实时可视化**：自定义 MapWidget 替代 RViz2 的部分功能，直接在界面内显示地图和机器人状态
3. **多线程架构**：ROS2 通信在后台线程运行，GUI 主线程保持流畅，通过 Qt 信号/槽安全通信
4. **进程安全设计**：SIGINT→SIGKILL 两阶段终止 + 进程组清理 + 启动时残留清理
5. **可扩展性**：通过 YAML 配置文件切换导航参数，支持添加新的场景和算法

### 架构中的设计模式

| 模式 | 应用 |
|------|------|
| **观察者模式** | Qt 信号/槽机制 |
| **生产者-消费者** | ROS 线程（生产数据）→ GUI 线程（消费显示） |
| **策略模式** | DWA / TEB 算法可插拔切换 |
| **外观模式** | ProcessManager 封装了所有 subprocess 操作 |

---

## 附录：环境安装速查

```bash
# 1. 安装 ROS2 Humble 桌面版（如果还没装）
# 参考: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# 2. 安装 TurtleBot3 仿真包
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-gazebo
sudo apt install ros-humble-turtlebot3-navigation2

# 3. 安装导航和建图工具
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-nav2-map-server

# 4. （可选）安装 TEB 避障算法
sudo apt install ros-humble-teb-local-planner

# 5. 安装 Python 依赖
pip3 install PyQt5 numpy --user

# 6. 设置环境变量（写入 ~/.bashrc）
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 7. 启动
cd /home/robot/07091430bugdefuse
./start_robot_gui.sh
```

---

*文档版本: v2.0 — 配合 robot_gui v1.0.0*
*最后更新: 2026-07-09*
*GitHub: [Fluxo-Lindage/turtlebot3-ros2-gui](https://github.com/Fluxo-Lindage/turtlebot3-ros2-gui)*
