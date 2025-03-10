# 机器狗上下位机通信控制方案

## 项目概述

本项目实现了机器狗上下位机之间的通信控制功能，使用户能够从上位机（Windows虚拟机）向下位机（机器狗主板）发送基本控制命令（如站立、行走等）。具体架构如下：

- **下位机**：机器狗主板，运行ROS1 (Noetic)，负责基础运动控制和硬件接口
- **上位机**：Windows系统下的VMware虚拟机，同样运行ROS1 (Noetic)，发送控制命令
- **通信方式**：通过机器人自带的WiFi网络，使用ROS1原生通信机制

## 项目结构

```
vmware_devq/
├── README.md                            # 本文档
├── 机器狗上下位机通信控制方案_虚拟机版.md    # 方案详细说明
├── robot_dog_control/                   # 上位机控制程序
│   ├── README.md                        # 上位机使用说明
│   ├── ros1_ws/                         # ROS工作空间
│   └── scripts/                         # 辅助脚本
│       ├── robot_control.sh             # 上位机控制脚本
│       └── setup_network.sh             # 网络配置脚本
└── devq_ws/                             # 下位机控制程序
    └── src/
        └── dev_robot_control_sdk/       # 机器狗控制SDK
            ├── robot_server.sh          # 下位机服务脚本
            ├── stand_up.sh              # 站立命令脚本
            ├── lie_down.sh              # 趴下命令脚本
            └── ...                      # 其他控制脚本
```

## 使用方法

### 一、网络连接设置

1. **连接方式**：
   - Windows主机连接到机器人创建的WiFi网络
   - 虚拟机配置为"桥接模式"，获取与宿主机相同网段的IP地址
   - 机器人默认IP地址：`10.10.10.10`
   - 虚拟机推荐IP地址：`10.10.10.20`（可通过`setup_network.sh`脚本配置）

2. **网络验证**：
   ```bash
   # 在虚拟机中测试与机器狗的连接
   ping 10.10.10.10
   ```

3. **ROS环境变量设置**：
   ```bash
   # 上位机设置（虚拟机）
   export ROS_MASTER_URI=http://10.10.10.10:11311
   export ROS_IP=10.10.10.20  # 替换为虚拟机实际IP
   
   # 下位机设置（已预配置）
   export ROS_MASTER_URI=http://10.10.10.10:11311
   export ROS_IP=10.10.10.10
   ```

### 二、下位机启动（先启动）

在使用上位机发送命令前，需要先在下位机（机器狗主板）上启动ROS控制程序：

```bash
# 连接到机器狗
ssh root@10.10.10.10

# 进入控制SDK目录
cd /path/to/devq_ws/src/dev_robot_control_sdk

# 启动下位机服务
chmod +x robot_server.sh
./robot_server.sh
```

这将启动下位机的ROS节点和控制程序，使其能够接收上位机发送的命令。脚本运行后，下位机将等待上位机发送控制指令。

### 三、上位机控制（后控制）

当下位机服务启动后，可以在上位机（虚拟机）发送控制命令：

```bash
# 进入控制脚本目录
cd robot_dog_control/scripts

# 赋予脚本执行权限
chmod +x robot_control.sh

# 发送控制命令
./robot_control.sh stand      # 机器狗站立
./robot_control.sh liedown    # 机器狗趴下
./robot_control.sh forward    # 机器狗前进
./robot_control.sh backward   # 机器狗后退
./robot_control.sh turn_left  # 机器狗左转
./robot_control.sh turn_right # 机器狗右转
./robot_control.sh stop       # 机器狗停止
```

## 通信原理

上位机通过ROS话题发送命令，下位机订阅这些话题并执行相应的动作：

1. **模式控制话题**: `/robot_control/set_mode`
   - 消息类型: `std_msgs/Int32MultiArray`
   - 模式对应关系:
     ```
     - 站立模式: data: [2]
     - 趴下模式: data: [1]
     - 被动模式: data: [0]
     - 强化学习模式: data: [3]
     - 软停止模式: data: [4]
     ```

2. **速度控制话题**: `/robot_control/set_velocity`
   - 消息类型: `std_msgs/Float32MultiArray`
   - 数据格式: [vx, vy, wz]
     ```
     - vx: 前进速度(m/s)，正值表示前进，负值表示后退
     - vy: 横向速度(m/s)，正值表示向左，负值表示向右
     - wz: 转向角速度(rad/s)，正值表示逆时针(左转)，负值表示顺时针(右转)
     ```
   - 常用命令示例:
     ```
     - 前进: data: [0.1, 0.0, 0.0]
     - 后退: data: [-0.1, 0.0, 0.0]
     - 左转: data: [0.0, 0.0, 0.3]
     - 右转: data: [0.0, 0.0, -0.3]
     - 停止: data: [0.0, 0.0, 0.0]
     ```

## 技术架构
```
+-------------------------+                +----------------------+
|        上位机           |                |       下位机         |
| (Windows VMware虚拟机)  |<-------------->|  (机器狗主板)        |
|       ROS1 Noetic       |  机器人WiFi网络 |    ROS1 Noetic       |
+-------------------------+                +----------------------+
|                         |                |                      |
| +---------------------+ |                | +------------------+ |
| |  robot_control.sh   | |    ROS话题通信   | |  robot_server.sh | |
| +---------------------+ |<-------------->| +------------------+ |
|                         |   /robot_control|                      |
| +---------------------+ |   /set_mode    | +------------------+ |
| |  setup_network.sh   | |   /set_velocity| |  基础运动控制模块  | |
| +---------------------+ |                | +------------------+ |
+-------------------------+                +----------------------+
```

## 脚本说明

### 上位机脚本

1. **robot_control.sh**
   - 作用：发送各种控制命令到下位机
   - 支持的命令：stand, liedown, forward, backward, turn_left, turn_right, stop
   - 位置：`robot_dog_control/scripts/robot_control.sh`

2. **setup_network.sh**
   - 作用：配置虚拟机网络参数，设置正确的IP地址和ROS环境变量
   - 位置：`robot_dog_control/scripts/setup_network.sh`

### 下位机脚本

1. **robot_server.sh**
   - 作用：启动下位机ROS控制程序，准备接收上位机命令
   - 位置：`devq_ws/src/dev_robot_control_sdk/robot_server.sh`
   - 功能：初始化ROS环境，启动控制节点，设置网络参数

2. **stand_up.sh**, **lie_down.sh**等
   - 作用：在下位机本地执行特定动作的快捷脚本
   - 位置：`devq_ws/src/dev_robot_control_sdk/`下的各个脚本文件

## 环境要求

### 上位机环境
- Windows系统（Windows 10或更高版本）
- VMware Workstation（16或更高版本）
- 虚拟机：Ubuntu 20.04
- ROS Noetic完整安装
- 网络：能够连接到机器狗WiFi

### 下位机环境
- 机器狗主板系统（已预装）
- ROS Noetic（已预装）
- 所需依赖包（已预装）

## 故障排除

1. **网络连接问题**
   - 确保Windows主机已连接到机器狗WiFi
   - 检查虚拟机网络设置是否为"桥接模式"
   - 验证虚拟机IP是否在10.10.10.x网段
   - 使用ping命令测试连接

2. **ROS通信问题**
   - 检查ROS_MASTER_URI和ROS_IP设置
   - 尝试使用`rostopic list`查看可用话题
   - 确认下位机服务正在运行

3. **命令执行问题**
   - 确保下位机先启动服务
   - 检查命令格式是否正确
   - 查看下位机控制程序是否有报错

## 资源链接

更多详细信息，请参考：

- [机器狗上下位机通信控制方案_虚拟机版.md](机器狗上下位机通信控制方案_虚拟机版.md)：详细的设计方案
- [robot_dog_control/README.md](robot_dog_control/README.md)：上位机使用说明
- [devq_ws/src/dev_robot_control_sdk/ROS使用说明.md](devq_ws/src/dev_robot_control_sdk/ROS使用说明.md)：下位机ROS接口说明