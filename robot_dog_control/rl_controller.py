#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import torch
import numpy as np
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import time
import os
import datetime
import signal
import sys
import traceback
import socket
import argparse

class RobotDogRLController:
    def __init__(self, model_path):
        # 用于控制程序结束
        self.should_terminate = False
        
        # 添加策略执行控制标志
        self.should_run_policy = False  # 默认不执行策略推理，直到完成站立流程
        
        # 设置日志文件
        log_dir = os.path.dirname(os.path.abspath(__file__))
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 创建日志目录
        log_folder = os.path.join(log_dir, f"logs_{timestamp}")
        os.makedirs(log_folder, exist_ok=True)
        
        # 分别设置三种不同的日志文件
        self.robot_state_log = os.path.join(log_folder, "robot_state_log.txt")
        self.command_log = os.path.join(log_folder, "command_log.txt")
        self.general_log = os.path.join(log_folder, "general_log.txt")
        
        # 为每个日志文件写入头部信息
        self.write_to_log(self.robot_state_log, f"=== 机器狗状态数据日志 - {timestamp} ===\n")
        self.write_to_log(self.command_log, f"=== 机器狗控制命令日志 - {timestamp} ===\n")
        self.write_to_log(self.general_log, f"=== 机器狗一般信息日志 - {timestamp} ===\n")
        
        # 日志计数器和频率控制
        self.log_counter = 0
        self.log_frequency = 100  # 每100次状态更新记录一次状态信息
        
        # 记录日志目录
        self.log_general(f"日志文件保存在目录: {log_folder}")
        self.log_general(f"模型路径: {model_path}")
        
        # ===== 网络诊断和设置 =====
        self.log_general("开始网络诊断...")
        
        # 列出所有网络接口和IP地址
        try:
            import subprocess
            # 显示所有网络接口
            self.log_general("获取所有网络接口信息:")
            ip_cmd = "ip addr show" if os.name != "nt" else "ipconfig /all"
            result = subprocess.run(ip_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            ip_info = result.stdout.decode('utf-8', errors='ignore')
            # 只记录摘要信息，避免日志过大
            ip_summary = "\n".join([line for line in ip_info.split("\n") if "inet " in line or "IPv4" in line])
            self.log_general(f"网络接口信息摘要:\n{ip_summary}")
        except Exception as e:
            self.log_general(f"获取网络接口信息失败: {e}")
        
        # 设置ROS网络环境变量，连接到机器狗下位机
        os.environ['ROS_MASTER_URI'] = 'http://10.10.10.10:11311'
        
        # 尝试ping机器狗，测试网络连通性
        self.log_general("测试与机器狗的网络连通性...")
        try:
            ping_cmd = "ping -c 2 10.10.10.10" if os.name != "nt" else "ping -n 2 10.10.10.10"
            result = subprocess.run(ping_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            ping_output = result.stdout.decode('utf-8', errors='ignore')
            if "bytes from" in ping_output or "字节，来自" in ping_output:
                self.log_general("网络连通性测试成功，可以ping通机器狗")
            else:
                self.log_general("警告: 无法ping通机器狗IP 10.10.10.10，这可能导致控制失败")
        except Exception as e:
            self.log_general(f"网络连通性测试失败: {e}")
        
        # 设置本机IP，确保不使用本地回环地址
        try:
            # 使用与robot_control.sh相同的获取IP方式
            import subprocess
            # 尝试使用hostname -I命令获取非回环IP
            result = subprocess.run("hostname -I", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            ip_output = result.stdout.decode('utf-8', errors='ignore').strip()
            ip_list = ip_output.split()
            
            # 如果有多个IP，优先选择10.10.10.x网段的IP，因为这是与机器狗通信的网段
            valid_ip = None
            for ip in ip_list:
                if ip.startswith("10.10.10."):
                    valid_ip = ip
                    break
            
            # 如果没找到10.10网段的IP，则使用第一个非回环IP
            if not valid_ip and ip_list:
                for ip in ip_list:
                    if not ip.startswith("127."):
                        valid_ip = ip
                        break
            
            # 如果仍然没找到或命令失败，使用默认配置IP
            if not valid_ip:
                # 明确设置为固定IP 10.10.10.20
                valid_ip = '10.10.10.20'
                self.log_general(f"未找到有效IP，使用默认IP: {valid_ip}")
            else:
                self.log_general(f"成功获取非回环IP: {valid_ip}")
            
            os.environ['ROS_IP'] = valid_ip
        except Exception as e:
            # 如果自动获取失败，使用默认IP
            os.environ['ROS_IP'] = '10.10.10.20'
            self.log_general(f"获取IP失败，使用默认IP: 10.10.10.20, 错误: {e}")
        
        self.log_general(f"最终ROS_IP设置为: {os.environ.get('ROS_IP', '未设置')}")
        self.log_general(f"ROS_MASTER_URI设置为: {os.environ.get('ROS_MASTER_URI', '未设置')}")
        
        rospy.init_node('robot_dog_rl_controller', anonymous=True)

        # 加载PyTorch模型
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = torch.load(model_path, map_location=self.device)
        self.model.eval()
        self.log_general(f"模型已加载到{self.device}设备")
        
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
        
        # 观察历史 - 修改为235维以匹配模型期望的输入维度
        self.obs_history = np.zeros(235)  # 模型期望235维输入
        
        # 初始化标志
        self.initialized = False
        
        # 设置信号处理器
        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.log_general("等待ROS连接...")
        self.interruptible_sleep(1)  # 等待连接建立
        self.log_general("ROS连接就绪")
    
    def write_to_log(self, log_file, message):
        """将消息写入指定的日志文件"""
        try:
            with open(log_file, 'a', encoding='utf-8') as f:
                f.write(f"{message}\n")
        except Exception as e:
            print(f"写入日志文件时出错: {e}")
    
    def log_robot_state(self, message):
        """记录机器人状态信息"""
        print(f"[状态] {message}")
        self.write_to_log(self.robot_state_log, f"[{datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]}] {message}")
    
    def log_command(self, message):
        """记录发送给机器人的命令"""
        print(f"[命令] {message}")
        self.write_to_log(self.command_log, f"[{datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]}] {message}")
    
    def log_general(self, message):
        """记录一般信息"""
        print(f"[信息] {message}")
        self.write_to_log(self.general_log, f"[{datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]}] {message}")
    
    def signal_handler(self, sig, frame):
        """处理Ctrl+C信号，安全停止机器狗"""
        self.log_general("\n收到中断信号，正在安全停止机器狗...")
        # 停止所有运动
        self.set_velocity_command(0.0, 0.0, 0.0)
        # 切换到软停止模式
        self.set_mode(4)  # 软停止模式
        self.interruptible_sleep(1)
        # 标记程序应该终止
        self.should_terminate = True
        self.log_general("已发送停止命令。程序即将退出...")
        sys.exit(0)
    
    def interruptible_sleep(self, duration):
        """可中断的sleep函数，能响应Ctrl+C"""
        start_time = time.time()
        while time.time() - start_time < duration:
            if self.should_terminate or rospy.is_shutdown():
                return
            # 使用短的sleep间隔，以便能及时响应中断
            time.sleep(0.1)
        
    def robot_state_callback(self, robot_state_msg):
        """处理机器人状态更新回调"""
        try:
            # 将消息数据转换为Numpy数组
            self.robot_state = np.array(robot_state_msg.data, dtype=np.float32)
            self.last_state_time = time.time()
            
            # 记录机器人状态数据
            if self.log_counter % self.log_frequency == 0:
                # 记录选择性状态信息
                base_position = self.robot_state[0:3]
                base_orientation = self.robot_state[3:7]
                joint_states = []
                for i in range(12):
                    pos = self.robot_state[7 + i*2]
                    vel = self.robot_state[7 + i*2 + 1]
                    joint_states.append((pos, vel))
                
                # 仅记录部分重要信息，避免日志过大
                log_data = f"Time: {time.time():.3f}, Pos: {base_position}, Orient: {base_orientation[:2]}"
                self.log_robot_state(log_data)
            
            self.log_counter += 1
            
            # 只有当机器人已经成功站立并启用策略时，才执行策略推理
            if not self.initialized:
                self.initialized = True
                self.log_general("初始化完成，接收到第一个机器人状态")
            elif self.should_run_policy:  # 只有当应该运行策略时才执行
                # 执行模型推理
                with torch.no_grad():
                    # 预处理状态数据
                    state_tensor = torch.from_numpy(self.robot_state).to(self.device).unsqueeze(0)
                    
                    # 模型推理
                    action = self.model(state_tensor)
                    
                    # 后处理动作输出
                    action_np = action.squeeze().cpu().numpy()
                    
                    # 发送命令到机器人
                    self.set_control_command(action_np)
        
        except Exception as e:
            self.log_general(f"处理机器人状态时发生错误: {str(e)}")
            traceback.print_exc()
    
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
        
        # 使用单值模式，与robot_control.sh和robot_server_rl.sh匹配
        mode_msg.data = [mode]
        
        # 记录详细的命令信息
        self.log_command(f"发送模式命令: data: [{mode}]")
        
        # 重复发布多次确保命令被接收
        success = False
        for i in range(5):  # 增加尝试次数到5次
            try:
                self.mode_pub.publish(mode_msg)
                self.log_command(f"第{i+1}次发布模式命令")
                success = True
            except Exception as e:
                self.log_general(f"发布模式命令失败: {e}")
            self.interruptible_sleep(0.2)  # 增加等待时间
        
        if success:
            self.log_command(f"设置机器人模式: {mode} 成功")
        else:
            self.log_general(f"警告: 可能未成功设置机器人模式: {mode}")
    
    def setup_remote_control(self):
        """配置远程控制模式，适用于robot_server_rl.sh环境"""
        self.log_general("设置远程控制模式...")
        mode_msg = Int32MultiArray()
        # 匹配robot_server_rl.sh中的远程控制模式设置: "data: [1, 1]"
        mode_msg.data = [1, 1]
        self.mode_pub.publish(mode_msg)
        self.interruptible_sleep(1)
        self.log_command("已设置远程控制模式")
    
    def set_velocity_command(self, vx, vy, wz):
        """设置期望的速度命令"""
        self.commands = np.array([vx, vy, wz])
        vel_msg = Float32MultiArray()
        vel_msg.data = [vx, vy, wz]
        self.vel_pub.publish(vel_msg)
        self.log_command(f"设置速度命令: vx={vx}, vy={vy}, wz={wz}")
    
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
            
        # 记录关节位置和速度数据
        self.log_robot_state(f"关节位置: {joint_positions}")
        self.log_robot_state(f"关节速度: {joint_velocities}")
        
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
        
        # 打印当前观察向量维度（调试用）
        self.log_general(f"基础观察向量维度: {len(obs)}")
        
        # 记录观察向量的详细内容
        self.log_general(f"观察向量详细内容:")
        self.log_general(f"  命令信号 (3维): {obs[0:3]}")
        self.log_general(f"  基本角速度 (3维): {obs[3:6]}")
        self.log_general(f"  四元数 (4维): {obs[6:10]}")
        self.log_general(f"  投影重力 (3维): {obs[10:13]}")
        self.log_general(f"  关节位置偏差 (12维): {obs[13:25]}")
        self.log_general(f"  关节速度 (12维): {obs[25:37]}")
        self.log_general(f"  上一步动作 (12维): {obs[37:49]}")
        
        # 临时解决方案：创建固定大小235维的观察向量并填充
        # 原始观察向量只有49维，但模型期望235维输入
        full_obs = np.zeros(235)  # 创建235维的零向量
        
        # 复制已有观察数据到新向量中
        for i in range(min(len(obs), 235)):
            full_obs[i] = obs[i]
            
        # 这里用0填充缺失的地形高度测量数据（约186维）
        # 无需额外填充，因为full_obs已经初始化为零向量
        
        self.log_general(f"填充后观察向量维度: {len(full_obs)}")
        return full_obs
    
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
        if self.should_terminate:
            return
            
        # 构建观察向量
        obs = self.build_observation_vector()
        if obs is None:
            return
            
        # 确保观察向量维度正确
        if len(obs) != 235:
            self.log_general(f"警告: 观察向量维度 {len(obs)} 不符合期望的 235")
            # 紧急修复，如果维度仍然不对，手动调整
            temp_obs = np.zeros(235)
            for i in range(min(len(obs), 235)):
                temp_obs[i] = obs[i]
            obs = temp_obs
            
        # 直接使用当前观察向量（已填充到235维）
        self.log_general(f"最终输入张量维度: {obs.shape}")
        obs_tensor = torch.FloatTensor(obs).unsqueeze(0).to(self.device)
        self.log_general(f"送入模型的张量维度: {obs_tensor.shape}")
        
        try:
            with torch.no_grad():
                actions = self.model(obs_tensor).cpu().numpy()[0]
            
            # 输出动作数组维度
            self.log_general(f"模型输出动作维度: {actions.shape}")
            
            # 提取关节位置控制命令
            if len(actions) >= 15:
                # 原始处理方式
                joint_positions = actions[3:15]
            else:
                # 适应较小的动作数组
                if len(actions) > 3:
                    joint_positions = actions[3:] if len(actions) > 3 else actions
                    # 如果动作少于12个，填充到12个
                    if len(joint_positions) < 12:
                        joint_positions = np.pad(joint_positions, 
                                            (0, 12-len(joint_positions)), 
                                            'constant')
                else:
                    # 如果动作数组太小，使用零数组
                    self.log_general("警告: 动作数组太小，使用零数组")
                    joint_positions = np.zeros(12)
            
            self.last_actions = joint_positions
            
            # 计算目标关节位置
            target_positions = []
            for i in range(12):
                target_positions.append(self.default_positions[i] + self.action_scale * joint_positions[i])
            
            # 记录目标关节位置
            self.log_command(f"目标关节位置: {target_positions}")
            
            # 发送关节位置命令
            joint_msg = Float32MultiArray()
            joint_msg.data = target_positions
            self.joint_cmd_pub.publish(joint_msg)
        except Exception as e:
            self.log_general(f"模型推理或发送命令出错: {e}")
    
    def run_demo(self):
        """执行演示控制序列"""
        try:
            if not self.initialized:
                self.log_general("等待初始化完成...")
                timeout = time.time() + 30  # 30秒超时
                while not self.initialized and not rospy.is_shutdown() and not self.should_terminate:
                    if time.time() > timeout:
                        self.log_general("等待初始化超时！")
                        return
                    self.interruptible_sleep(0.1)
            
            if self.should_terminate:
                return
            
            # 显示ROS连接信息
            self.log_general("ROS连接信息:")
            self.log_general(f"  ROS_MASTER_URI: {os.environ.get('ROS_MASTER_URI')}")
            self.log_general(f"  ROS_IP: {os.environ.get('ROS_IP')}")
            
            # 完全使用与robot_control.sh脚本相同的方式发送站立命令
            import subprocess
            
            # 定义一个函数来执行rostopic命令
            def run_rostopic_cmd(cmd):
                self.log_general(f"执行命令: {cmd}")
                result = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                self.log_general(f"命令执行结果: {result.returncode}")
                self.log_general(f"标准输出: {result.stdout.decode('utf-8', errors='ignore')}")
                self.log_general(f"标准错误: {result.stderr.decode('utf-8', errors='ignore')}")
                return result.returncode == 0
            
            # 站立模式 (完全相同的命令)
            self.log_general("发送站立命令 (与robot_control.sh完全相同)...")
            success = run_rostopic_cmd("rostopic pub --once /robot_control/set_mode std_msgs/Int32MultiArray \"data: [2]\"")
            if not success:
                self.log_general("发送站立命令可能失败，尝试使用ROS接口...")
                self.set_mode(2)  # 站立模式
            
            self.log_general("等待机器人完成站立...")
            self.interruptible_sleep(8)  # 增加等待时间到8秒
            
            if self.should_terminate:
                return
            
            # 发送零速度命令稳定站立姿态
            self.log_general("发送零速度命令稳定站立姿态...")
            run_rostopic_cmd("rostopic pub --once /robot_control/set_velocity std_msgs/Float32MultiArray \"data: [0.0, 0.0, 0.0]\"")
            self.interruptible_sleep(3)  # 稳定站立3秒
            
            if self.should_terminate:
                return
            
            # 切换到强化学习模式
            self.log_general("切换到强化学习模式...")
            success = run_rostopic_cmd("rostopic pub --once /robot_control/set_mode std_msgs/Int32MultiArray \"data: [3]\"")
            if not success:
                self.log_general("发送强化学习模式命令可能失败，尝试使用ROS接口...")
                self.set_mode(3)  # 切换到强化学习模式
            
            self.interruptible_sleep(2)  # 等待模式切换
            
            # 检查站立状态
            self.log_general("站立完成，检查站立状态...")
            # 通过检查机器人当前的关节位置确认站立状态
            if self.robot_state is not None:
                joint_positions = []
                for i in range(12):
                    joint_positions.append(self.robot_state[7 + i*2])
                self.log_general(f"当前关节位置: {joint_positions}")
                self.log_general("站立状态检查完成")
            else:
                self.log_general("警告：无法获取机器人状态，无法确认站立状态")
            
            # 开始执行强化学习控制
            self.should_run_policy = True
            self.log_general("强化学习控制已启用，开始执行策略推理")
            
            # 主循环
            while not rospy.is_shutdown() and not self.should_terminate:
                self.interruptible_sleep(0.1)
                
        except Exception as e:
            self.log_general(f"演示控制序列发生错误: {str(e)}")
            traceback.print_exc()
            
        finally:
            # 确保在退出时设置为安全模式
            try:
                self.log_general("控制序列结束，切换到被动模式...")
                self.should_run_policy = False
                self.set_mode(0)  # 被动模式
            except:
                pass

def main():
    try:
        # 解析命令行参数
        parser = argparse.ArgumentParser(description='机器狗强化学习控制器')
        parser.add_argument('--ip', help='手动设置ROS_IP地址，例如：10.10.10.20')
        args = parser.parse_args()
        
        # 如果提供了IP参数，设置环境变量
        if args.ip:
            os.environ['ROS_IP'] = args.ip
            print(f"已手动设置ROS_IP: {args.ip}")
        
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rl_model/policy_1.pt")
        
        if not os.path.exists(model_path):
            print(f"错误: 模型文件 {model_path} 不存在！")
            exit(1)
            
        # 创建控制器实例
        controller = RobotDogRLController(model_path)
        
        # 运行演示
        controller.run_demo()
        
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"发生错误: {e}")

if __name__ == '__main__':
    main()
