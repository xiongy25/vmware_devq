/****************************************************************************** 
 * @ Dev Robot Control SDK
 * @ Copyright (c) 2024 WEILAN.Co.Ltd. All Rights Reserved.
******************************************************************************/
#include "robot_control/remote_control.h"
#include <memory.h>

RemoteControl::RemoteControl()
{
}

RemoteControl::~RemoteControl()
{
}

void RemoteControl::get_command(RemoteCommand &cmd)
{
    cmd_mutex_.lock();
    memcpy(&cmd, &cmd_, sizeof(RemoteCommand));
    cmd_mutex_.unlock();
}

RosRemoteControl::RosRemoteControl() : node_("robot_control")
{
    subscribers_.emplace_back(node_.subscribe("set_mode", 5, &RosRemoteControl::on_set_mode_callback, this));
    subscribers_.emplace_back(node_.subscribe("set_velocity", 5, &RosRemoteControl::on_set_velocity_callback, this));
}

RosRemoteControl::~RosRemoteControl()
{
}

void RosRemoteControl::initialize(RobotData *robot_data)
{
    robot_data_ = robot_data;
    subscribers_.emplace_back(node_.subscribe("set_joints", 5, &RosRemoteControl::on_set_joints_callback, this));
}

void RosRemoteControl::on_set_mode_callback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    std::cout << "[RosRemoteControl] set mode ";
    for (auto it = msg->data.begin(); it != msg->data.end(); it++)
    {
        std::cout << *it << ",";
    }
    std::cout << std::endl;

    cmd_mutex_.lock();
    for (int i = 0; i < std::min(3, static_cast<int>(msg->data.size())); ++i)
    {
        cmd_.mode[i] = msg->data[i];
    }
    cmd_mutex_.unlock();
}

void RosRemoteControl::on_set_velocity_callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    std::cout << "[RosRemoteControl] set velocity ";
    for (auto it = msg->data.begin(); it != msg->data.end(); it++)
    {
        std::cout << *it << ",";
    }
    std::cout << std::endl;

    int size = static_cast<int>(msg->data.size());
    cmd_mutex_.lock();
    if (size >= 1)
    {
        cmd_.vx = msg->data[0];

        if (size >= 2)
        {
            cmd_.vy = msg->data[1];

            if (size >= 3)
            {
                cmd_.wz = msg->data[2];
            }
        }
    }
    cmd_mutex_.unlock();
}

void RosRemoteControl::on_set_joints_callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if (robot_data_ == nullptr || cmd_.mode[0] != 3) {
        // 只在强化学习模式下接收关节命令
        return;
    }
    
    std::cout << "[RosRemoteControl] set joints" << std::endl;
    
    const int size = msg->data.size();
    if (size != robot_data_->robot_command.joints.size()) {
        ROS_ERROR("[RosRemoteControl] Received joint command with wrong size (%d vs %zu)", 
                size, robot_data_->robot_command.joints.size());
        return;
    }
    
    for (int i = 0; i < size; i++) {
        robot_data_->robot_command.joints[i].pos = msg->data[i];
        robot_data_->robot_command.joints[i].vel = 0.0f;
        robot_data_->robot_command.joints[i].tau = 0.0f;
        
        // 默认的kp和kd值
        if (robot_data_->robot_command.joints[i].kp == 0.0f) {
            robot_data_->robot_command.joints[i].kp = 50.0f;
        }
        if (robot_data_->robot_command.joints[i].kd == 0.0f) {
            robot_data_->robot_command.joints[i].kd = 1.0f;
        }
    }
    
    // 设置第二个模式为1，表示使用上位机发送的关节命令
    cmd_mutex_.lock();
    cmd_.mode[1] = 1;
    cmd_mutex_.unlock();
}
