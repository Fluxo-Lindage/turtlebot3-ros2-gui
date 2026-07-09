#!/bin/bash
# =============================================================================
# 机器人仿真控制平台 — 一键启动脚本
#
# 本脚本自动完成以下工作：
#   1. 加载 ROS2 Humble 环境
#   2. 设置 TurtleBot3 型号
#   3. 设置 Gazebo 模型路径
#   4. 启动 GUI 程序
#
# 使用方法:
#   chmod +x start_robot_gui.sh
#   ./start_robot_gui.sh
#
# 或者从任意位置运行:
#   bash /path/to/start_robot_gui.sh
# =============================================================================

set -e  # 遇到错误立即退出

echo "================================================"
echo "  机器人仿真控制平台 — 启动中..."
echo "================================================"
echo ""

# ---- 1. 加载 ROS2 环境 ----
echo "[1/5] 加载 ROS2 Humble 环境..."
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "  [OK] ROS2 Humble 环境已加载"
else
    echo "  [失败] 未找到 /opt/ros/humble/setup.bash"
    echo "     请确认 ROS2 Humble 已正确安装"
    exit 1
fi

# ---- 2. 设置 TurtleBot3 型号 ----
echo "[2/5] 设置 TurtleBot3 型号..."
export TURTLEBOT3_MODEL=waffle_pi
echo "  [OK] TURTLEBOT3_MODEL = ${TURTLEBOT3_MODEL}"

# ---- 3. 设置 Gazebo 模型路径 ----
echo "[3/5] 设置 Gazebo 模型路径..."
# 添加 TurtleBot3 模型路径
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:${GAZEBO_MODEL_PATH}
echo "  [OK] GAZEBO_MODEL_PATH 已设置"

# ---- 4. 获取项目根目录 ----
echo "[4/5] 定位项目目录..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "  [OK] 项目目录: ${SCRIPT_DIR}"

# ---- 5. 安装依赖（首次运行时需要） ----
echo "[5/5] 检查 Python 依赖..."
if ! python3 -c "import PyQt5" 2>/dev/null; then
    echo "  [警告] 未检测到 PyQt5，正在安装..."
    pip3 install PyQt5 --user
    echo "  [OK] PyQt5 安装完成"
else
    echo "  [OK] PyQt5 已安装"
fi

# 检查必要的 ROS2 包
echo ""
echo "检查 ROS2 功能包..."

check_ros_pkg() {
    if ros2 pkg list 2>/dev/null | grep -q "^$1$"; then
        echo "  [OK] $1"
    else
        echo "  [警告] $1 — 未安装，部分功能可能不可用"
    fi
}

check_ros_pkg "turtlebot3_gazebo"
check_ros_pkg "turtlebot3_navigation2"
check_ros_pkg "slam_toolbox"
check_ros_pkg "nav2_bringup"
check_ros_pkg "nav2_map_server"

# TEB 避障算法（非必需，DWA 已足够满足课程要求）
if ros2 pkg list 2>/dev/null | grep -q "teb_local_planner"; then
    echo "  [OK] teb_local_planner (TEB 避障算法可用)"
else
    echo "  [提示] TEB 算法需要从源码编译安装（ROS2 Humble 无预编译包）。"
    echo "     DWA 算法已内置，可正常满足避障需求。"
fi

echo ""
echo "================================================"
echo "  启动机器人仿真控制平台 GUI..."
echo "================================================"
echo ""
echo "  提示:"
echo "    1. 先点击「启动仿真」选择并启动 Gazebo 环境"
echo "    2. 然后点击「开始建图」启动 SLAM"
echo "    3. 建图完成后点击「保存地图」"
echo "    4. 加载地图后点击「开始导航」即可进行自主导航"
echo ""

cd "${SCRIPT_DIR}"
python3 -m robot_gui.main
