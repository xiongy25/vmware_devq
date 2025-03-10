#!/bin/bash
# 机器狗网络配置脚本

# 显示当前网络接口信息
echo "当前网络接口信息："
ip addr

# 检查是否连接到机器狗WiFi网络
echo "检查是否连接到机器狗WiFi网络..."
ping -c 3 10.10.10.10

if [ $? -eq 0 ]; then
    echo "已成功连接到机器狗WiFi网络！"
    
    # 设置ROS环境变量
    echo "设置ROS网络环境变量..."
    export ROS_MASTER_URI=http://10.10.10.10:11311
    export ROS_IP=$(hostname -I | awk '{print $1}')
    
    echo "ROS_MASTER_URI=$ROS_MASTER_URI"
    echo "ROS_IP=$ROS_IP"
    
    # 测试ROS连接
    echo "测试ROS连接..."
    rostopic list
    
    if [ $? -eq 0 ]; then
        echo "ROS连接成功！可以开始发送控制命令。"
    else
        echo "ROS连接失败，请检查ROS Master是否运行。"
    fi
else
    echo "无法连接到机器狗，请确保已连接到正确的WiFi网络。"
fi 