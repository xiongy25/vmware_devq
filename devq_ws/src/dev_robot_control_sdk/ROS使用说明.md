# ROS 使用说明

本项目使用 ROS（Robot Operating System）作为机器人控制的通信框架。以下是详细的 ROS 使用说明。

## ROS 依赖

项目使用了以下 ROS 包：

- `roscpp`：ROS C++ 客户端库
- `std_msgs`：标准消息类型
- `geometry_msgs`：几何相关的消息类型
- `sensor_msgs`：传感器相关的消息类型
- `message_generation`：消息生成工具
- `message_runtime`：消息运行时支持

## ROS 节点

项目创建了一个名为 `robot_control` 的 ROS 节点，用于处理机器人的远程控制功能。

### 订阅的话题（Subscribers）

1. **模式设置话题**
   - 话题名：`/robot_control/set_mode`
   - 消息类型：`std_msgs::Int32MultiArray`
   - 功能：设置机器人的运动模式
   - 数据格式：整数数组，表示不同的运动模式
     ```
     模式对应关系：
     0: PASSIVE（被动模式）
     1: LIE_DOWN（趴下模式）
     2: STAND_UP（站立模式）
     3: RL_MODEL（强化学习模式）
     4: SOFT_STOP（软停止模式）
     ```

2. **速度控制话题**
   - 话题名：`/robot_control/set_velocity`
   - 消息类型：`std_msgs::Float32MultiArray`
   - 功能：设置机器人的运动速度
   - 数据格式：浮点数数组 [vx, vy, wz]
     ```
     vx: 前进速度（m/s）
     vy: 横向速度（m/s）
     wz: 转向角速度（rad/s）
     ```

## ROS 通信示例

### 1. 设置运动模式

```bash
# 切换到站立模式
rostopic pub --once /robot_control/set_mode std_msgs/Int32MultiArray "data: [2]"

# 切换到趴下模式
rostopic pub --once /robot_control/set_mode std_msgs/Int32MultiArray "data: [1]"
```

### 2. 控制机器人速度

```bash
# 设置前进速度为0.1m/s，横向速度为0，转向角速度为0.3rad/s
rostopic pub --once /robot_control/set_velocity std_msgs/Float32MultiArray "data: [0.1, 0.0, 0.3]"

# 停止移动
rostopic pub --once /robot_control/set_velocity std_msgs/Float32MultiArray "data: [0.0, 0.0, 0.0]"
```

## 代码实现说明

1. **远程控制类**
   - `RemoteControl`：基类，提供基本的远程控制接口
   - `RosRemoteControl`：ROS实现的远程控制类，继承自`RemoteControl`

2. **回调函数**
   - `on_set_mode_callback`：处理模式设置指令
   - `on_set_velocity_callback`：处理速度控制指令

3. **线程安全**
   - 使用互斥锁（mutex）保护命令数据的读写操作
   - 确保多线程环境下的数据一致性

## 注意事项

1. **消息发布频率**
   - 订阅者队列大小设置为5，避免消息堆积
   - 建议控制指令发布频率不超过200Hz

2. **安全控制**
   - 在发送速度指令前，确保机器人处于正确的运动模式
   - 使用软停止模式（SOFT_STOP）来安全地停止机器人

3. **调试方法**
   - 可以使用 `rostopic echo` 命令监听话题数据
   - 使用 `rqt_graph` 查看节点通信关系
   - 使用 `rosnode info` 查看节点详细信息

## 扩展开发

如果需要添加新的ROS功能，可以：

1. 在 `package.xml` 中添加新的依赖包
2. 创建新的消息类型（如需要）
3. 在 `RosRemoteControl` 类中添加新的订阅者或发布者
4. 实现相应的回调函数
