# 机器狗上位机控制系统

本项目实现了从Windows虚拟机上位机向机器狗下位机发送控制命令的功能。通过机器狗自带的WiFi网络建立连接，实现基本的运动控制。

## 文件结构

```
robot_dog_control/
├── README.md                 # 本文档
├── rl_controller.py          # 强化学习控制器
├── rl_model/                 # 强化学习模型文件夹
│   └── policy_1.pt           # PyTorch模型文件
├── ros1_ws/                  # ROS工作空间
│   └── src/
│       └── robot_command_publisher/   # 命令发布包
│           ├── CMakeLists.txt         # 编译配置
│           ├── package.xml            # 包信息
│           ├── include/               # 头文件
│           └── src/                   # 源代码
│               └── command_publisher.cpp  # 命令发布节点
└── scripts/                   # 辅助脚本
    ├── robot_control.sh       # 机器狗控制脚本
    └── setup_network.sh       # 网络配置脚本
```

## 使用说明

### 环境要求

- VMware虚拟机运行Ubuntu 20.04
- ROS1 Noetic
- 已安装std_msgs、roscpp等依赖包
- Python 3.8+
- PyTorch 1.8+

### 配置步骤

1. 在Windows主机上连接到机器狗WiFi网络
2. 确保虚拟机网络配置为"桥接模式"
3. 在虚拟机中运行网络配置脚本：
   ```bash
   cd robot_dog_control/scripts
   chmod +x setup_network.sh
   ./setup_network.sh
   ```

### 编译ROS包

```bash
cd robot_dog_control/ros1_ws
catkin_make
source devel/setup.bash
```

### 发送控制命令

#### 上位机操作
使用控制脚本发送命令：

```bash
cd robot_dog_control/scripts
chmod +x robot_control.sh
./robot_control.sh stand      # 机器狗站立
./robot_control.sh liedown    # 机器狗趴下
./robot_control.sh forward    # 机器狗前进
./robot_control.sh backward   # 机器狗后退
./robot_control.sh turn_left  # 机器狗左转
./robot_control.sh turn_right # 机器狗右转
./robot_control.sh stop       # 机器狗停止
```

#### 下位机操作
在使用上位机发送命令前，需要在下位机上启动机器人控制服务：

```bash
cd /path/to/devq_ws/src/dev_robot_control_sdk
chmod +x robot_server.sh
./robot_server.sh
```

这将启动下位机ROS节点和控制程序，准备接收上位机的命令。确保下位机服务正在运行，否则上位机发送的命令将无法被执行。

### 验证连接

设置完成后，可以使用以下命令验证配置是否正确：

```bash
# 查看IP地址
ip addr

# 测试与机器狗的连接
ping 10.10.10.10
```

确保你选择的IP地址（如10.10.10.20）不与网络中其他设备冲突。你可以根据需要选择10.10.10.x网段中的任何未使用的地址（其中x为2-254之间的数字，避开已使用的IP如10.10.10.10）。

## 网络配置

### 基本网络信息
- 机器狗IP地址：10.10.10.10
- ROS_MASTER_URI=http://10.10.10.10:11311
- 虚拟机应获取与宿主机相同网段的IP地址（10.10.10.x）

### 通信原理

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

3. **机器人状态和关节控制话题** (强化学习模式):
   - 状态话题: `/robot_control/robot_state`
     - 消息类型: `std_msgs/Float32MultiArray`
     - 数据格式: [gyro_x, gyro_y, gyro_z, quat_w, quat_x, quat_y, quat_z, joint1_pos, joint1_vel, ...]
   
   - 关节控制话题: `/robot_control/set_joints`
     - 消息类型: `std_msgs/Float32MultiArray`
     - 数据格式: [joint1_pos, joint2_pos, ..., joint12_pos]

4. **命令执行流程**:
   - 下位机启动ROS节点并订阅相关话题
   - 上位机通过发布命令到指定话题控制机器狗
   - 下位机接收到命令后执行相应动作
   - 命令执行全过程不需要下位机额外操作

### 上位机IP地址设置方法

#### 如何查看上位机IP地址

在设置ROS网络环境变量前，您需要先查看上位机的IP地址。在Ubuntu终端中，可以使用以下任一命令查看：

```bash
# 方法1：使用ip命令
ip addr

# 方法2：使用ifconfig命令(如果未安装，使用 sudo apt install net-tools 安装)
ifconfig
```

在命令输出中，找到与机器狗在同一网段的IP地址（通常是10.10.10.x）。例如：
```
2: ens33: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc fq_codel state UP group default qlen 1000
    link/ether 00:0c:29:d5:b7:bf brd ff:ff:ff:ff:ff:ff
    altname enp2s1
    inet 10.10.10.97/24 brd 10.10.10.255 scope global dynamic noprefixroute ens33
```
在这个例子中，上位机IP地址是 **10.10.10.97**。

上位机（虚拟机）的IP地址需要设置在与机器狗相同的网段内（10.10.10.x），以下是三种设置方法：

#### 方法一：在虚拟机中设置静态IP

1. 在Ubuntu虚拟机中打开终端，创建或编辑网络配置文件：
   ```bash
   sudo nano /etc/netplan/01-network-manager-all.yaml
   ```

2. 添加以下配置（注意替换网卡名称，可以通过`ifconfig`命令查看）：
   ```yaml
   network:
     version: 2
     renderer: NetworkManager
     ethernets:
       ens33:  # 你的网卡名称可能不同，如eth0
         dhcp4: no
         addresses: [10.10.10.20/24]  # 设置静态IP，避免使用10.10.10.10（已被机器狗使用）
         gateway4: 10.10.10.1
         nameservers:
           addresses: [8.8.8.8, 8.8.4.4]
   ```

3. 应用网络配置：
   ```bash
   sudo netplan apply
   ```

#### 方法二：使用VMware网络设置

1. 在VMware中，确保虚拟机的网络连接模式设置为"桥接模式"
2. 选择桥接到连接机器狗WiFi的网卡
3. 复制物理网络连接状态（勾选）

#### 方法三：通过图形界面设置（Ubuntu 20.04）

1. 点击右上角网络图标
2. 选择"有线设置"或"WiFi设置"
3. 点击设置图标（齿轮）打开网络连接设置
4. 选择IPv4选项卡
5. 将"方法"改为"手动"
6. 添加IP地址：10.10.10.20，子网掩码：255.255.255.0，网关：10.10.10.1
7. 添加DNS服务器：8.8.8.8
8. 点击"应用"保存设置

### 验证连接

设置完成后，可以使用以下命令验证配置是否正确：

```bash
# 查看IP地址
ip addr

# 测试与机器狗的连接
ping 10.10.10.10
```

确保你选择的IP地址（如10.10.10.20）不与网络中其他设备冲突。你可以根据需要选择10.10.10.x网段中的任何未使用的地址（其中x为2-254之间的数字，避开已使用的IP如10.10.10.10）。

## 强化学习模型迁移到上位机

为了提高机器狗的控制性能，我们将强化学习模型从下位机迁移到了上位机运行。这种架构允许我们在上位机上运行更复杂、计算开销更大的控制算法，同时利用下位机处理低级别的硬件控制。

### 架构说明

![强化学习控制架构](https://mermaid.ink/img/pako:eNptkc9uwjAMxl8l8nlI5QFyQNqkHrZJ29C0w9QLappoaROlCYhR3n1OKRQYuTjf7_tjyx4BVZaAQ8GrhtXPtVAGQjPkPLsXdU3KTERGLOmktBIaCBNXkBIbCEZUP-YclLYwedezIxVXKzw7Xgmud0XoQAw5LdnW2o-Ufd9fEVvdUmaPl63dOZOt5Qqmjmw9Fx-ijOEzxCv1N_9lAOFwAe0eihvcO2Z-VnCDC0P5_9iMTVq0rWw5cJnLTu2ARwXzb_OPMkDOyYZLDsfKohNEXI0a1pJbfbGmF1Vd7_QW4K0VNbWybtQrvHdNweChbtFDHvFXwrsdKB1RDyVlSPuQUOy_0eEwPcWDFHBWh28zOSjSKDyIwsXOPIrhcZxM-_0-eYzjKRxVw0eXCm2F?type=png)

**系统组成**:
1. **上位机**:
   - 运行PyTorch强化学习模型
   - 通过ROS话题接收机器人状态数据
   - 计算并发送关节位置命令
   
2. **下位机**:
   - 发布机器人状态数据
   - 接收关节控制命令
   - 执行底层硬件控制

### 下位机代码修改

为了支持这种架构，我们对下位机代码进行了以下修改：

1. **添加了状态发布功能**:
   - 创建了`RosStatePublisher`类，用于发布机器狗状态数据
   - 发布的数据包括基本线速度、角速度、姿态四元数和关节状态

2. **修改了远程控制模块**:
   - 在`RosRemoteControl`类中添加了关节命令订阅功能
   - 添加了处理来自上位机的关节位置命令的回调函数

3. **修改了强化学习状态机**:
   - 在`FSMStateRLModel`中添加了对上位机控制模式的支持
   - 当指定为上位机控制模式时，跳过本地模型推理

### 上位机代码功能

上位机控制程序(`rl_controller.py`)的主要功能：

1. **加载和运行PyTorch模型**:
   - 从文件加载预训练的强化学习模型
   - 使用GPU（如果可用）加速模型推理

2. **ROS通信**:
   - 订阅下位机发布的机器人状态数据
   - 发布计算得到的关节控制命令
   - 发布模式和速度控制命令

3. **演示功能**:
   - 提供一个简单的演示序列，控制机器狗执行一系列动作
   - 包括站立、前进、转向等基本动作

### 使用方法

#### 环境准备
确保你的上位机安装了以下软件包：
```bash
sudo apt update
sudo apt install python3-pip ros-noetic-ros-base
pip3 install torch numpy
```

#### 运行强化学习控制器
```bash
# 设置ROS环境
export ROS_MASTER_URI=http://10.10.10.10:11311
export ROS_IP=10.10.10.20  # 替换为你的上位机IP

# 运行控制器
cd /home/ubuntu22/vmware_devq/robot_dog_control
chmod +x rl_controller.py
./rl_controller.py
```

#### 控制器工作流程
1. 控制器启动后会先让机器狗站立
2. 然后切换到强化学习模式(模式3)
3. 执行一系列演示动作（前进、左转、右转）
4. 最后回到站立模式

### 配置和调整

你可以根据需要修改`rl_controller.py`中的参数：

- **模型路径**: 更改`model_path`变量指向你的自定义模型
- **演示序列**: 修改`run_demo()`方法中的动作序列
- **控制参数**: 调整速度、控制增益等参数

### 故障排除

- 如果模型加载失败，检查模型文件路径是否正确
- 如果无法接收机器人状态数据，检查ROS连接和话题名称
- 如果机器狗动作异常，可能需要调整控制参数或检查模型兼容性

## 故障排除

- 如果无法连接到机器狗WiFi，请检查WiFi连接状态
- 如果ROS通信失败，检查ROS_MASTER_URI和ROS_IP配置 