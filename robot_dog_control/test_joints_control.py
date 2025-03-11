#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试关节控制功能的简单脚本
这个脚本会将机器狗切换到强化学习模式，然后通过关节位置控制接口发送简单的摆动命令
"""

import rospy
import numpy as np
import time
from std_msgs.msg import Float32MultiArray, Int32MultiArray

def main():
    # 初始化ROS节点
    rospy.init_node('joint_control_test', anonymous=True)
    
    # 创建发布者
    mode_pub = rospy.Publisher('/robot_control/set_mode', Int32MultiArray, queue_size=10)
    joint_cmd_pub = rospy.Publisher('/robot_control/set_joints', Float32MultiArray, queue_size=10)
    
    # 等待发布者连接
    time.sleep(1)
    print("发布者初始化完成")
    
    # 设置消息
    mode_msg = Int32MultiArray()
    joint_msg = Float32MultiArray()
    
    # 首先让机器狗站立
    print("切换到站立模式...")
    mode_msg.data = [2, 0, 0]  # 站立模式
    mode_pub.publish(mode_msg)
    time.sleep(5)  # 等待机器狗站立
    
    # 切换到强化学习模式
    print("切换到强化学习模式...")
    mode_msg.data = [3, 0, 0]  # 强化学习模式
    mode_pub.publish(mode_msg)
    time.sleep(2)  # 等待切换完成
    
    # 关节默认位置 (与FSMStateRLModel中的target_pose一致)
    default_positions = [0.0, 0.7, -1.4,  # 右前腿
                         0.0, 0.7, -1.4,  # 左前腿
                         0.0, 0.7, -1.4,  # 右后腿
                         0.0, 0.7, -1.4]  # 左后腿
    
    # 测试关节控制 - 进行简单的摆动
    print("开始关节控制测试...")
    rate = rospy.Rate(10)  # 10Hz
    
    for i in range(50):  # 运行5秒
        # 创建一个简单的正弦波摆动，修改右前腿和左前腿的膝关节
        joint_positions = default_positions.copy()
        joint_positions[2] = -1.4 + 0.3 * np.sin(i * 0.4)  # 右前腿膝关节
        joint_positions[5] = -1.4 + 0.3 * np.sin(i * 0.4 + np.pi)  # 左前腿膝关节
        
        # 发布关节位置命令
        joint_msg.data = joint_positions
        joint_cmd_pub.publish(joint_msg)
        
        print(f"第 {i+1}/50 步: 发送关节命令")
        rate.sleep()
    
    # 结束测试，回到站立模式
    print("测试完成，切换回站立模式...")
    mode_msg.data = [2, 0, 0]  # 站立模式
    mode_pub.publish(mode_msg)
    time.sleep(3)  # 等待机器狗恢复站立姿势
    
    print("测试脚本执行完成！")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
