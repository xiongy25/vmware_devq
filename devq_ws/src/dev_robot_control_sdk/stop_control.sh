#!/bin/bash

# 设置错误时退出
set -e

echo "正在查找并终止机器人控制程序..."

# 检查是否存在运行中的机器人控制程序
if pgrep -f "robot_control" > /dev/null; then
    pkill -f "robot_control"
    echo "已成功停止机器人控制程序"
else
    echo "未发现正在运行的机器人控制程序"
fi 