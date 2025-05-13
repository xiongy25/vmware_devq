#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import subprocess
import time
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import datetime
import traceback

def setup_ros_environment():
    """设置ROS环境"""
    try:
        # 检查ROS环境变量
        if 'ROS_MASTER_URI' not in os.environ:
            os.environ['ROS_MASTER_URI'] = 'http://localhost:11311'
        if 'ROS_HOSTNAME' not in os.environ:
            os.environ['ROS_HOSTNAME'] = 'localhost'
            
        # 设置ROS环境
        ros_setup_script = '/opt/ros/noetic/setup.bash'
        if not os.path.exists(ros_setup_script):
            print(f"错误：找不到ROS环境设置脚本 {ros_setup_script}")
            print("请确保ROS已正确安装")
            sys.exit(1)
            
        # 检查roscore是否运行
        try:
            subprocess.check_output(['rosnode', 'list'], stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError:
            print("正在启动roscore...")
            # 启动roscore
            roscore_process = subprocess.Popen(['roscore'])
            time.sleep(3)  # 等待roscore启动
            
        print("ROS环境设置完成")
        
    except Exception as e:
        print(f"设置ROS环境时发生错误: {e}")
        sys.exit(1)

class JointTester:
    def __init__(self):
        # 设置ROS环境
        setup_ros_environment()
        
        # 设置日志文件
        log_dir = os.path.dirname(os.path.abspath(__file__))
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_folder = os.path.join(log_dir, f"joint_test_logs_{timestamp}")
        os.makedirs(log_folder, exist_ok=True)
        self.log_file = os.path.join(log_folder, "joint_test_log.txt")
        
        # 初始化ROS节点
        rospy.init_node('joint_tester', anonymous=True)
        
        # 创建发布者
        self.mode_pub = rospy.Publisher('/robot_control/set_mode', Int32MultiArray, queue_size=10)
        self.joint_cmd_pub = rospy.Publisher('/robot_control/set_joints', Float32MultiArray, queue_size=10)
        
        # 初始化状态变量
        self.robot_state = None
        
        # 从配置文件读取默认位置
        self.default_positions = [
            0.0, -0.73, 1.25,  # 左前腿：髋关节,大腿关节,小腿关节
            0.0, -0.73, 1.25,  # 右前腿：髋关节,大腿关节,小腿关节
            0.0, -0.73, 1.25,  # 左后腿：髋关节,大腿关节,小腿关节
            0.0, -0.73, 1.25   # 右后腿：髋关节,大腿关节,小腿关节
        ]
        
        # 关节名称映射（根据URDF顺序）
        self.joint_names = {
            0: "左前髋关节",
            1: "左前大腿关节",
            2: "左前小腿关节",
            3: "右前髋关节",
            4: "右前大腿关节",
            5: "右前小腿关节",
            6: "左后髋关节",
            7: "左后大腿关节",
            8: "左后小腿关节",
            9: "右后髋关节",
            10: "右后大腿关节",
            11: "右后小腿关节"
        }
        
        # 关节安全限制
        self.joint_limits = {
            "hip": (-0.5, 0.5),    # 髋关节限制
            "thigh": (-1.0, 0.0),  # 大腿关节限制
            "calf": (0.5, 2.0)     # 小腿关节限制
        }
        
        self.log("关节测试程序已启动")
        
    def log(self, message):
        """记录日志"""
        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        log_message = f"[{timestamp}] {message}"
        print(log_message)
        with open(self.log_file, 'a', encoding='utf-8') as f:
            f.write(f"{log_message}\n")
            
    def set_mode(self, mode):
        """设置机器人模式"""
        mode_msg = Int32MultiArray()
        mode_msg.data = [mode]
        self.mode_pub.publish(mode_msg)
        self.log(f"设置模式: {mode}")
        
    def send_joint_positions(self, positions):
        """发送关节位置命令"""
        joint_cmd_msg = Float32MultiArray()
        joint_cmd_msg.data = positions
        self.joint_cmd_pub.publish(joint_cmd_msg)
        self.log(f"发送关节位置: {positions}")
        
    def get_joint_type(self, joint_index):
        """根据关节索引获取关节类型"""
        if joint_index % 3 == 0:
            return "hip"
        elif joint_index % 3 == 1:
            return "thigh"
        else:
            return "calf"
            
    def test_joint(self, joint_index, amplitude=0.2):
        """测试指定关节
        
        Args:
            joint_index: 要测试的关节索引
            amplitude: 运动幅度（弧度）
        """
        # 获取当前关节名称和类型
        joint_name = self.joint_names.get(joint_index, f"未知关节 {joint_index}")
        joint_type = self.get_joint_type(joint_index)
        joint_limits = self.joint_limits[joint_type]
        
        self.log(f"\n开始测试 {joint_name} (索引: {joint_index}, 类型: {joint_type})")
        
        # 创建目标位置数组
        target_positions = self.default_positions.copy()
        
        # 计算目标位置，确保在安全范围内
        target_pos = target_positions[joint_index] + amplitude
        target_pos = max(min(target_pos, joint_limits[1]), joint_limits[0])
        
        # 只移动指定的关节
        target_positions[joint_index] = target_pos
        
        # 发送命令
        self.send_joint_positions(target_positions)
        self.log(f"已将 {joint_name} 移动到 {target_pos:.2f} 弧度")
        
        # 等待用户观察并确认
        while True:
            response = input(f"请观察机器人，{joint_name} 是否在运动？(y/n): ").lower()
            if response == 'y':
                # 用户确认正确，记录映射关系
                self.log(f"确认 {joint_name} (索引: {joint_index}) 映射正确")
                break
            elif response == 'n':
                # 用户确认不正确，提示输入实际运动的关节编号
                print("\n当前测试的关节信息：")
                print(f"关节名称: {joint_name}")
                print(f"关节索引: {joint_index}")
                print(f"关节类型: {joint_type}")
                print("\n请观察机器人，输入实际运动的关节编号 (0-11):")
                for idx, name in self.joint_names.items():
                    print(f"{idx}: {name}")
                
                while True:
                    try:
                        actual_index = int(input("请输入实际运动的关节编号: "))
                        if 0 <= actual_index <= 11:
                            self.log(f"用户确认 {joint_name} (索引: {joint_index}) 实际对应 {self.joint_names[actual_index]} (索引: {actual_index})")
                            break
                        else:
                            print("请输入0-11之间的数字")
                    except ValueError:
                        print("请输入有效的数字")
                break
            else:
                print("请输入 y 或 n")
        
        # 恢复默认位置
        self.send_joint_positions(self.default_positions)
        self.log(f"已将 {joint_name} 恢复到默认位置")
        
    def run_test(self):
        """运行交互式测试程序"""
        try:
            # 等待用户准备
            input("请确保机器人已上电，按回车开始测试...")
            
            # 设置站立模式
            self.set_mode(2)  # 站立模式
            time.sleep(8)  # 等待机器人站立
            
            # 测试所有关节
            for joint_index in range(12):
                self.test_joint(joint_index)
                
            self.log("\n测试完成！")
            self.log("请查看日志文件确认每个关节的映射关系。")
            
        except Exception as e:
            self.log(f"测试过程中发生错误: {e}")
            traceback.print_exc()
            
        finally:
            # 确保机器人回到安全状态
            self.set_mode(0)  # 被动模式

def main():
    tester = JointTester()
    tester.run_test()

if __name__ == '__main__':
    main() 