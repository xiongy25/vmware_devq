#!/bin/bash
# 机器狗控制脚本 - 改进版
# 移除了依赖不存在话题的检查

# 确保ROS环境已设置
source /opt/ros/noetic/setup.bash
source ~/ros1_ws/devel/setup.bash

# 设置ROS网络环境变量
export ROS_MASTER_URI=http://10.10.10.10:11311
export ROS_IP=$(hostname -I | awk '{print $1}')

# 切换到强化学习模式
switch_to_rl_mode() {
  echo "切换到强化学习模式..."
  rostopic pub --once /robot_control/set_mode std_msgs/Int32MultiArray "data: [3]"
  echo "等待模式切换完成..."
  sleep 2
}

# 准备速度控制 - 不再尝试检查当前模式
prepare_for_velocity() {
  if [ "$1" = "check_stand" ]; then
    echo "确保机器狗处于站立状态..."
    echo "先切换到站立模式..."
    rostopic pub --once /robot_control/set_mode std_msgs/Int32MultiArray "data: [2]"
    echo "等待机器狗完成站立..."
    sleep 5
  fi
  
  # 直接切换到强化学习模式，不检查当前模式
  switch_to_rl_mode
}

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
  "rl_mode")
    echo "切换到强化学习模式..."
    switch_to_rl_mode
    ;;
  "forward")
    prepare_for_velocity "check_stand"
    echo "发送前进命令..."
    rostopic pub --once /robot_control/set_velocity std_msgs/Float32MultiArray "data: [0.1, 0.0, 0.0]"
    ;;
  "backward")
    prepare_for_velocity "check_stand"
    echo "发送后退命令..."
    rostopic pub --once /robot_control/set_velocity std_msgs/Float32MultiArray "data: [-0.1, 0.0, 0.0]"
    ;;
  "turn_left")
    prepare_for_velocity "check_stand"
    echo "发送左转命令..."
    rostopic pub --once /robot_control/set_velocity std_msgs/Float32MultiArray "data: [0.0, 0.0, 0.3]"
    ;;
  "turn_right")
    prepare_for_velocity "check_stand"
    echo "发送右转命令..."
    rostopic pub --once /robot_control/set_velocity std_msgs/Float32MultiArray "data: [0.0, 0.0, -0.3]"
    ;;
  "forward_left")
    prepare_for_velocity "check_stand"
    echo "发送前进并左转命令..."
    rostopic pub --once /robot_control/set_velocity std_msgs/Float32MultiArray "data: [0.1, 0.0, 0.2]"
    ;;
  "forward_right")
    prepare_for_velocity "check_stand"
    echo "发送前进并右转命令..."
    rostopic pub --once /robot_control/set_velocity std_msgs/Float32MultiArray "data: [0.1, 0.0, -0.2]"
    ;;
  "stop")
    prepare_for_velocity
    echo "发送停止命令..."
    rostopic pub --once /robot_control/set_velocity std_msgs/Float32MultiArray "data: [0.0, 0.0, 0.0]"
    ;;
  *)
    echo "用法: $0 [command]"
    echo ""
    echo "可用命令:"
    echo "  stand         - 机器狗站立"
    echo "  liedown       - 机器狗趴下"
    echo "  rl_mode       - 切换到强化学习模式"
    echo "  forward       - 机器狗前进 (自动切换到强化学习模式)"
    echo "  backward      - 机器狗后退 (自动切换到强化学习模式)"
    echo "  turn_left     - 机器狗左转 (自动切换到强化学习模式)"
    echo "  turn_right    - 机器狗右转 (自动切换到强化学习模式)"
    echo "  forward_left  - 机器狗前进并左转 (自动切换到强化学习模式)"
    echo "  forward_right - 机器狗前进并右转 (自动切换到强化学习模式)"
    echo "  stop          - 机器狗停止 (自动切换到强化学习模式)"
    echo ""
    echo "注意: 所有速度控制命令会自动切换到强化学习模式"
    exit 1
    ;;
esac 