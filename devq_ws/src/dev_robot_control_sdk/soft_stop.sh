#!/bin/bash

# 设置错误时退出
set -e

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# 定义清理函数
cleanup() {
    echo "正在关闭所有运行中的机器人控制程序..."
    pkill -f "robot_control" || echo "未发现正在运行的机器人控制程序"
    exit 0
}

# 捕获 SIGINT 信号
trap cleanup SIGINT

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/noetic/setup.bash" ]; then
        source /opt/ros/noetic/setup.bash
    else
        echo "错误：找不到ROS环境配置文件 (/opt/ros/noetic/setup.bash)"
        exit 1
    fi
fi

# 检查工作空间是否需要编译
if [ ! -d "${WORKSPACE_DIR}/build" ] || [ ! -d "${WORKSPACE_DIR}/install" ]; then
    echo "检测到工作空间未编译，开始编译..."
    
    # 保存当前目录
    CURRENT_DIR=$(pwd)
    
    # 切换到工作空间目录
    cd "${WORKSPACE_DIR}"
    
    # 执行编译
    if ! catkin_make; then
        echo "错误：工作空间编译失败"
        cd "${CURRENT_DIR}"
        exit 1
    fi
    
    echo "工作空间编译成功"
    
    # 返回原目录
    cd "${CURRENT_DIR}"
fi

# 检查编译输出
if [ ! -f "${WORKSPACE_DIR}/devel/setup.bash" ]; then
    echo "错误：找不到工作空间的setup.bash文件，编译可能不完整"
    exit 1
fi

# 设置工作空间环境
source "${WORKSPACE_DIR}/devel/setup.bash"

# 检查并进入控制程序目录
CONTROL_DIR="${WORKSPACE_DIR}/install/lib/robot_control"
if [ -d "${CONTROL_DIR}" ]; then
    cd "${CONTROL_DIR}"
else
    echo "错误：找不到控制程序目录 (${CONTROL_DIR})"
    exit 1
fi

# 设置库路径
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib:./

# 检查配置文件
if [ ! -f "config/devq.yaml" ]; then
    echo "错误：找不到配置文件 (config/devq.yaml)"
    exit 1
fi

# 检查是否已经有实例在运行
if pgrep -f "robot_control" > /dev/null; then
    echo "警告：发现已有机器人控制程序在运行"
else
    # 启动机器人控制程序（在后台运行）
    ./robot_control config/devq.yaml &
    echo "已启动机器人控制程序"
fi

# 等待几秒钟确保控制程序完全启动
echo "等待控制程序启动..."
sleep 5

# 检查ROS主节点是否运行
if ! rostopic list > /dev/null 2>&1; then
    echo "错误：ROS主节点未运行"
    exit 1
fi

# 发送软停止命令
echo "发送软停止命令..."
if rostopic pub --once /robot_control/set_mode std_msgs/Int32MultiArray "data: [4]" > /dev/null 2>&1; then
    echo "已成功发送软停止命令到机器人"
else
    echo "错误：发送软停止命令失败"
    exit 1
fi 