#!/bin/bash
# 机器狗控制脚本

# 确保ROS环境已设置
source /opt/ros/noetic/setup.bash
source ~/ros1_ws/devel/setup.bash

# 设置ROS网络环境变量
export ROS_MASTER_URI=http://10.10.10.10:11311
export ROS_IP=$(hostname -I | awk '{print $1}')

# 根据参数发送命令
case "$1" in
  "stand")
    echo "发送站立命令..."
    rostopic pub --once /robot_control/set_mode std_msgs/Int32MultiArray "data: [2]"
    ;;
  "liedown")
    echo "发送趴下命令..."
    rostopic pub --once /robot_control/set_mode std_msgs/Int32MultiArray "data: [1]"
    ;;
  "forward")
    echo "发送前进命令..."
    rostopic pub --once /robot_control/set_velocity std_msgs/Float32MultiArray "data: [0.1, 0.0, 0.0]"
    ;;
  "backward")
    echo "发送后退命令..."
    rostopic pub --once /robot_control/set_velocity std_msgs/Float32MultiArray "data: [-0.1, 0.0, 0.0]"
    ;;
  "turn_left")
    echo "发送左转命令..."
    rostopic pub --once /robot_control/set_velocity std_msgs/Float32MultiArray "data: [0.0, 0.0, 0.3]"
    ;;
  "turn_right")
    echo "发送右转命令..."
    rostopic pub --once /robot_control/set_velocity std_msgs/Float32MultiArray "data: [0.0, 0.0, -0.3]"
    ;;
  "stop")
    echo "发送停止命令..."
    rostopic pub --once /robot_control/set_velocity std_msgs/Float32MultiArray "data: [0.0, 0.0, 0.0]"
    ;;
  *)
    echo "用法: $0 [stand|liedown|forward|backward|turn_left|turn_right|stop]"
    exit 1
    ;;
esac 