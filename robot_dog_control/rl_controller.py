#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import torch
import numpy as np
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import time
import os

class RobotDogRLController:
    def __init__(self, model_path):
        # 设置ROS网络环境变量，连接到机器狗下位机
        os.environ['ROS_MASTER_URI'] = 'http://10.10.10.10:11311'
        # 设置本机IP (请根据实际情况修改)
        # os.environ['ROS_IP'] = '192.168.1.100'  # 取消注释并修改为您的计算机IP
        
        rospy.init_node('robot_dog_rl_controller', anonymous=True)

        # 加载PyTorch模型
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = torch.load(model_path, map_location=self.device)
        self.model.eval()
        print(f"模型已加载到{self.device}设备")
        
        # 观察空间和动作空间缩放因子
        self.obs_scales = {
            'lin_vel': 2.0,
            'ang_vel': 0.25,
            'dof_pos': 1.0,
            'dof_vel': 0.05,
            'height_measurements': 5.0
        }
        self.action_scale = 0.5  # 与下位机配置一致
        
        # 创建发布者用于发送控制命令
        self.mode_pub = rospy.Publisher('/robot_control/set_mode', Int32MultiArray, queue_size=10)
        self.joint_cmd_pub = rospy.Publisher('/robot_control/set_joints', Float32MultiArray, queue_size=10)
        self.vel_pub = rospy.Publisher('/robot_control/set_velocity', Float32MultiArray, queue_size=10)
        
        # 创建订阅者接收机器人状态
        self.robot_state_sub = rospy.Subscriber(
            '/robot_control/robot_state', 
            Float32MultiArray, 
            self.robot_state_callback
        )
        
        # 初始化状态变量
        self.robot_state = None
        self.last_actions = np.zeros(12)  # 上一步动作
        self.commands = np.zeros(3)       # 当前命令 [vx, vy, wz]
        
        # 关节默认位置
        self.default_positions = [0.0, 0.7, -1.4, 0.0, 0.7, -1.4, 0.0, 0.7, -1.4, 0.0, 0.7, -1.4]
        
        # 观察历史
        self.obs_history = np.zeros(49 * 10)  # 49个观察值，10个时间步
        
        # 初始化标志
        self.initialized = False
        
        print("等待ROS连接...")
        time.sleep(1)  # 等待连接建立
        print("ROS连接就绪")
        
    def robot_state_callback(self, msg):
        """接收并处理机器人状态数据"""
        self.robot_state = np.array(msg.data)
        
        # 记录初始化状态
        if not self.initialized and self.robot_state is not None:
            self.initialized = True
            print("收到机器人状态数据，初始化完成")
        
        # 如果收到状态数据，运行模型推理
        if self.robot_state is not None:
            self.run_policy_inference()
    
    def set_mode(self, mode):
        """设置机器人模式
        模式:
          0 - 被动模式
          1 - 趴下
          2 - 站立
          3 - 强化学习模式
          4 - 软停止
        """
        mode_msg = Int32MultiArray()
        mode_msg.data = [mode, 0, 0]
        self.mode_pub.publish(mode_msg)
        print(f"已将机器人切换到模式: {mode}")
    
    def set_velocity_command(self, vx, vy, wz):
        """设置期望的速度命令"""
        self.commands = np.array([vx, vy, wz])
        vel_msg = Float32MultiArray()
        vel_msg.data = [vx, vy, wz]
        self.vel_pub.publish(vel_msg)
        print(f"设置速度命令: vx={vx}, vy={vy}, wz={wz}")
    
    def build_observation_vector(self):
        """构建与训练时相同格式的观察向量"""
        if self.robot_state is None:
            return None
            
        # 提取状态数据
        gyro_x, gyro_y, gyro_z = self.robot_state[0:3]
        quat_w, quat_x, quat_y, quat_z = self.robot_state[3:7]
        
        # 提取关节位置和速度
        joint_positions = []
        joint_velocities = []
        for i in range(12):
            joint_positions.append(self.robot_state[7 + i*2])
            joint_velocities.append(self.robot_state[7 + i*2 + 1])
            
        # 计算投影重力向量（与FSMStateRLModel中相同的方法）
        quat = np.array([quat_w, quat_x, quat_y, quat_z])
        projected_gravity = self.quat_rotate_inverse(quat, np.array([0, 0, -1]))
        
        # 构建单步观察向量
        obs = []
        
        # 命令信号
        obs.extend(self.commands)
        
        # 基本角速度
        obs.extend([
            gyro_x * self.obs_scales['ang_vel'],
            gyro_y * self.obs_scales['ang_vel'],
            gyro_z * self.obs_scales['ang_vel']
        ])
        
        # 四元数
        obs.extend([quat_w, quat_x, quat_y, quat_z])
        
        # 投影重力
        obs.extend(projected_gravity)
        
        # 关节位置偏差
        for i in range(12):
            obs.append((joint_positions[i] - self.default_positions[i]) * self.obs_scales['dof_pos'])
        
        # 关节速度
        for i in range(12):
            obs.append(joint_velocities[i] * self.obs_scales['dof_vel'])
        
        # 上一步动作
        obs.extend(self.last_actions)
        
        return np.array(obs)
    
    def quat_rotate_inverse(self, q, v):
        """四元数旋转的逆变换，与FSMStateRLModel中相同的实现"""
        q_w = q[0]
        q_vec = q[1:4]
        a = v * (2.0 * q_w * q_w - 1.0)
        b = np.cross(q_vec, v) * q_w * 2.0
        c = q_vec * (np.dot(q_vec, v)) * 2.0
        return a - b + c
    
    def run_policy_inference(self):
        """运行策略模型推理并发送控制命令"""
        # 构建观察向量
        obs_step = self.build_observation_vector()
        if obs_step is None:
            return
            
        # 更新观察历史
        self.obs_history = np.roll(self.obs_history, -len(obs_step))
        self.obs_history[-len(obs_step):] = obs_step
        
        # 转换为PyTorch张量并进行模型推理
        obs_tensor = torch.FloatTensor(self.obs_history).unsqueeze(0).to(self.device)
        
        with torch.no_grad():
            actions = self.model(obs_tensor).cpu().numpy()[0]
        
        # 提取关节位置控制命令
        joint_positions = actions[3:15]  # 假设前3个是速度，后面12个是关节位置
        self.last_actions = joint_positions
        
        # 计算目标关节位置
        target_positions = []
        for i in range(12):
            target_positions.append(self.default_positions[i] + self.action_scale * joint_positions[i])
        
        # 发送关节位置命令
        joint_msg = Float32MultiArray()
        joint_msg.data = target_positions
        self.joint_cmd_pub.publish(joint_msg)
    
    def run_demo(self):
        """执行演示控制序列"""
        if not self.initialized:
            print("等待初始化完成...")
            while not self.initialized and not rospy.is_shutdown():
                time.sleep(0.1)
        
        print("正在切换到站立模式...")
        self.set_mode(2)  # 先让机器人站立
        time.sleep(5)     # 等待站立完成
        
        print("正在切换到强化学习模式...")
        self.set_mode(3)  # 切换到强化学习模式
        time.sleep(2)     # 等待模式切换
        
        print("开始前进...")
        self.set_velocity_command(0.5, 0.0, 0.0)  # 设置前进速度
        time.sleep(5)                             # 前进5秒
        
        print("开始左转...")
        self.set_velocity_command(0.3, 0.0, 0.3)   # 左转
        time.sleep(3)                             # 左转3秒
        
        print("开始右转...")
        self.set_velocity_command(0.3, 0.0, -0.3)  # 右转
        time.sleep(3)                             # 右转3秒
        
        print("停止...")
        self.set_velocity_command(0.0, 0.0, 0.0)   # 停止
        time.sleep(2)                             # 停止2秒
        
        print("切换回站立模式...")
        self.set_mode(2)  # 切换回站立模式
        
        print("演示完成")

if __name__ == '__main__':
    try:
        model_path = "/home/ubuntu22/vmware_devq/robot_dog_control/rl_model/policy_1.pt"
        
        if not os.path.exists(model_path):
            print(f"错误: 模型文件 {model_path} 不存在！")
            exit(1)
            
        controller = RobotDogRLController(model_path)
        
        # 运行演示
        controller.run_demo()
        
        # 或者进入ROS循环，处理回调
        # rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"发生错误: {e}")
