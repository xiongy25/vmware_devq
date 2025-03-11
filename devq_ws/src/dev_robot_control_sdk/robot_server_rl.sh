#!/bin/bash

# 机器狗下位机RL模式服务脚本
# 用途：启动下位机控制程序并接收上位机强化学习模型的控制命令

# 设置错误时退出
set -e

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"  # 自动获取工作空间路径

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

# 检查工作空间
if [ ! -d "${WORKSPACE_DIR}" ]; then
    echo "错误：工作空间目录不存在，请修改脚本中的WORKSPACE_DIR变量"
    exit 1
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

# 设置工作空间环境
if [ -f "${WORKSPACE_DIR}/devel/setup.bash" ]; then
    source "${WORKSPACE_DIR}/devel/setup.bash"
else
    echo "错误：找不到工作空间的setup.bash文件，编译可能不完整"
    exit 1
fi

# 检查并进入控制程序目录
CONTROL_DIR="${WORKSPACE_DIR}/devel/lib/robot_control"
if [ ! -d "${CONTROL_DIR}" ]; then
    echo "警告：找不到devel目录下的控制程序目录 (${CONTROL_DIR})"
    # 尝试在其他可能的位置查找
    CONTROL_DIR="${WORKSPACE_DIR}/install/lib/robot_control"
    if [ ! -d "${CONTROL_DIR}" ]; then
        # 尝试直接在源码目录查找
        CONTROL_DIR="${WORKSPACE_DIR}/src/dev_robot_control_sdk"
        if [ ! -d "${CONTROL_DIR}" ]; then
            echo "错误：无法找到控制程序目录"
            exit 1
        fi
    fi
fi
echo "使用控制程序目录: ${CONTROL_DIR}"
cd "${CONTROL_DIR}"

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
    else
        echo "错误：找不到控制程序可执行文件"
        exit 1
    fi
else
    ROBOT_EXEC="./robot_control"
fi

# 设置ROS网络环境变量 - 确保下位机能与上位机通信
# 下位机IP (通常是10.10.10.10)
export ROS_IP=10.10.10.10
export ROS_MASTER_URI=http://10.10.10.10:11311

# 检查是否已经有实例在运行
if pgrep -f "robot_control" > /dev/null; then
    echo "警告：发现已有机器人控制程序在运行"
    # 获取现有进程的PID
    CONTROL_PID=$(pgrep -f "robot_control" | head -1)
    echo "使用现有的控制程序 (PID: ${CONTROL_PID})"
else
    # 启动机器人控制程序 - 使用远程控制模式
    echo "启动机器人控制程序（远程控制模式）..."
    # 添加远程控制模式参数 - 适用于强化学习模型控制
    ${ROBOT_EXEC} config/devq.yaml --remote_mode=true &
    CONTROL_PID=$!
    echo "已启动机器人控制程序 (PID: ${CONTROL_PID})"
    
    # 等待几秒钟确保控制程序完全启动
    echo "等待控制程序启动..."
    sleep 5
    
    # 检查进程是否仍在运行
    if ! ps -p ${CONTROL_PID} > /dev/null 2>&1; then
        echo "错误：控制程序启动失败或已崩溃"
        echo "请检查日志以获取更多信息"
        exit 1
    fi
fi

# 检查ROS主节点是否运行
if ! rostopic list > /dev/null 2>&1; then
    echo "错误：ROS主节点未运行"
    echo "尝试启动ROS主节点..."
    roscore &
    sleep 3
fi

# 设置RL远程控制模式
echo "设置远程控制模式为RL模式..."
# 发送模式切换命令 - 假设已有一个set_remote_mode话题
rostopic pub -1 /robot_control/set_mode std_msgs/Int32MultiArray "data: [1, 1]" || echo "警告：无法设置远程控制模式，可能需要人工设置"

echo ""
echo "============================================="
echo "机器狗下位机RL控制模式已启动"
echo "已配置为接收上位机强化学习模型的控制命令"
echo "支持的话题："
echo "- 状态发布: /robot/state"
echo "- 命令接收: /robot/command"
echo "============================================="
echo "按 Ctrl+C 可以停止服务并退出"
echo ""

# 显示一些有用的调试信息
echo "已注册的ROS话题:"
rostopic list

# 无限循环，保持脚本运行，直到用户按Ctrl+C
while true; do
    sleep 1
done
