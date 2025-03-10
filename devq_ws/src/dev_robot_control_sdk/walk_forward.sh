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

# 设置环境变量
if [ -f "${WORKSPACE_DIR}/devel/setup.bash" ]; then
    source "${WORKSPACE_DIR}/devel/setup.bash"
elif [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    source "${WORKSPACE_DIR}/install/setup.bash"
else
    echo "错误：找不到工作空间的setup.bash文件"
    exit 1
fi

# 设置库路径
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib:./

# 检查配置文件
if [ ! -f "config/devq.yaml" ]; then
    echo "警告：当前目录下找不到配置文件 (config/devq.yaml)"
    # 尝试在其他可能的位置查找
    if [ -f "${WORKSPACE_DIR}/src/dev_robot_control_sdk/config/devq.yaml" ]; then
        echo "在源码目录找到配置文件，复制到当前目录"
        mkdir -p config
        cp "${WORKSPACE_DIR}/src/dev_robot_control_sdk/config/devq.yaml" config/
    else
        echo "错误：找不到配置文件 (config/devq.yaml)"
        exit 1
    fi
fi

# 检查模型目录
if [ ! -d "model/devq" ] || [ ! -f "model/devq/policy.mnn" ]; then
    echo "警告：当前目录下找不到模型文件 (model/devq/policy.mnn)"
    
    # 检查单数model目录
    if [ -f "${WORKSPACE_DIR}/src/dev_robot_control_sdk/model/devq/policy.mnn" ]; then
        echo "在源码目录找到模型文件，复制到当前目录"
        mkdir -p model/devq
        cp "${WORKSPACE_DIR}/src/dev_robot_control_sdk/model/devq/policy.mnn" model/devq/
    # 检查复数models目录（可能是命名不一致）
    elif [ -f "${WORKSPACE_DIR}/src/dev_robot_control_sdk/models/devq/policy.mnn" ]; then
        echo "在源码目录的models目录找到模型文件，复制到当前目录"
        mkdir -p model/devq
        cp "${WORKSPACE_DIR}/src/dev_robot_control_sdk/models/devq/policy.mnn" model/devq/
    else
        echo "错误：找不到模型文件 (model/devq/policy.mnn)"
        echo "请确保源码目录中存在model/devq/policy.mnn或models/devq/policy.mnn文件"
        exit 1
    fi
fi

# 检查可执行文件
if [ ! -f "./robot_control" ]; then
    echo "警告：当前目录下找不到控制程序可执行文件 (robot_control)"
    # 尝试在devel目录查找
    if [ -f "${WORKSPACE_DIR}/devel/lib/robot_control/robot_control" ]; then
        echo "在devel目录找到可执行文件，使用该文件"
        ROBOT_EXEC="${WORKSPACE_DIR}/devel/lib/robot_control/robot_control"
    # 尝试在install目录查找
    elif [ -f "${WORKSPACE_DIR}/install/lib/robot_control/robot_control" ]; then
        echo "在install目录找到可执行文件，使用该文件"
        ROBOT_EXEC="${WORKSPACE_DIR}/install/lib/robot_control/robot_control"
    else
        echo "错误：找不到控制程序可执行文件"
        exit 1
    fi
else
    ROBOT_EXEC="./robot_control"
fi

# 检查是否已经有实例在运行
CONTROL_RUNNING=false
if pgrep -f "robot_control" > /dev/null; then
    echo "警告：发现已有机器人控制程序在运行"
    # 获取现有进程的PID
    CONTROL_PID=$(pgrep -f "robot_control" | head -1)
    echo "使用现有的控制程序 (PID: ${CONTROL_PID})"
    CONTROL_RUNNING=true
else
    # 启动机器人控制程序（在后台运行）
    echo "启动机器人控制程序..."
    ${ROBOT_EXEC} config/devq.yaml &
    CONTROL_PID=$!
    echo "已启动机器人控制程序 (PID: ${CONTROL_PID})"
fi

# 等待几秒钟确保控制程序完全启动
echo "等待控制程序启动..."
sleep 5

# 只有当我们启动了新实例时才检查进程是否仍在运行
if [ "$CONTROL_RUNNING" = false ] && ! ps -p ${CONTROL_PID} > /dev/null 2>&1; then
    echo "错误：控制程序启动失败或已崩溃"
    echo "请检查日志以获取更多信息"
    exit 1
fi

# 检查ROS主节点是否运行
if ! rostopic list > /dev/null 2>&1; then
    echo "错误：ROS主节点未运行"
    exit 1
fi

# 首先让机器人站立起来
echo "设置机器人为站立模式..."
if rostopic pub --once /robot_control/set_mode std_msgs/Int32MultiArray "data: [2]" > /dev/null 2>&1; then
    echo "已成功发送站立命令到机器人"
else
    echo "错误：发送站立命令失败"
    exit 1
fi

# 等待机器人完成站立动作
echo "等待机器人完成站立..."
sleep 5

# 然后切换到强化学习模型控制模式
echo "设置机器人为强化学习模型控制模式..."
if rostopic pub --once /robot_control/set_mode std_msgs/Int32MultiArray "data: [3]" > /dev/null 2>&1; then
    echo "已成功发送强化学习模型控制命令到机器人"
else
    echo "错误：发送强化学习模型控制命令失败"
    exit 1
fi

# 等待机器人完成模式切换
echo "等待机器人完成模式切换..."
sleep 3

# 设置前进速度 - 增加速度值并循环发送命令
echo "开始发送前进命令..."
FORWARD_SPEED=0.2  # 速度值保持不变
LATERAL_SPEED=0.0
ROTATION_SPEED=0.0

# 先发送一次速度命令
if rostopic pub --once /robot_control/set_velocity std_msgs/Float32MultiArray "data: [${FORWARD_SPEED}, ${LATERAL_SPEED}, ${ROTATION_SPEED}]" > /dev/null 2>&1; then
    echo "已成功发送前进命令到机器人（前进速度：${FORWARD_SPEED}）"
else
    echo "错误：发送前进命令失败"
    exit 1
fi

# 保持脚本运行，等待用户按Ctrl+C，并循环发送速度命令
echo ""
echo "机器人正在前进中..."
echo "按 Ctrl+C 可以停止机器人并退出"
echo ""
echo "提示：你可以通过在脚本中修改 FORWARD_SPEED 值来改变前进速度"
echo ""

# 无限循环发送速度命令，直到用户按Ctrl+C
COUNTER=0
while true; do
    # 每10秒重新发送一次速度命令，确保机器人持续运动
    if [ $((COUNTER % 10)) -eq 0 ]; then
        echo "重新发送速度命令以维持运动..."
        if ! rostopic pub --once /robot_control/set_velocity std_msgs/Float32MultiArray "data: [${FORWARD_SPEED}, ${LATERAL_SPEED}, ${ROTATION_SPEED}]" > /dev/null 2>&1; then
            echo "警告：发送速度命令失败"
        fi
    fi
    sleep 1
    COUNTER=$((COUNTER + 1))
done
