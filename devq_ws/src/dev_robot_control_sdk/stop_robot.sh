#!/bin/bash

echo "正在停止所有机器人控制程序..."

# 尝试使用pkill终止所有robot_control进程
if pkill -f "robot_control"; then
    echo "成功终止机器人控制程序"
else
    echo "未发现正在运行的机器人控制程序，或无法终止它们"
    
    # 尝试使用更强制的方式终止
    if pkill -9 -f "robot_control"; then
        echo "使用强制信号成功终止机器人控制程序"
    fi
fi

# 检查是否还有robot_control进程在运行
if pgrep -f "robot_control" > /dev/null; then
    echo "警告：仍有机器人控制程序在运行"
    echo "运行中的进程PID："
    pgrep -f "robot_control"
    echo "您可能需要手动终止这些进程"
else
    echo "所有机器人控制程序已停止"
fi
