/******************************************************************************
 * @ Dev Robot Control SDK
 * @ Copyright (c) 2024 WEILAN.Co.Ltd. All Rights Reserved.
 ******************************************************************************/
#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "robot_control/robot_data.h"

/**
 * ROS publisher for robot state
 */
class RosStatePublisher {
public:
    /**
     * Constructor method of class RosStatePublisher.
     * @param robot_data   Robot data.
     */
    RosStatePublisher(RobotData *robot_data) : robot_data_(robot_data) {}

    /**
     * Initialize ROS publisher.
     * @param node   ROS node handle.
     * @return 0 for success, others for failure.
     */
    int initialize(ros::NodeHandle &node) {
        robot_state_pub_ = node.advertise<std_msgs::Float32MultiArray>("/robot_control/robot_state", 10);
        return 0;
    }

    /**
     * Publish robot state.
     * @return 0 for success, others for failure.
     */
    int publish_state() {
        std_msgs::Float32MultiArray state_msg;
        
        // 基本线速度和角速度
        state_msg.data.push_back(robot_data_->robot_state.base.gyro_x);
        state_msg.data.push_back(robot_data_->robot_state.base.gyro_y);
        state_msg.data.push_back(robot_data_->robot_state.base.gyro_z);
        
        // 姿态四元数
        state_msg.data.push_back(robot_data_->robot_state.base.quaternion_w);
        state_msg.data.push_back(robot_data_->robot_state.base.quaternion_x);
        state_msg.data.push_back(robot_data_->robot_state.base.quaternion_y);
        state_msg.data.push_back(robot_data_->robot_state.base.quaternion_z);
        
        // 关节状态
        for (int i = 0; i < robot_data_->robot_state.joints.size(); i++) {
            state_msg.data.push_back(robot_data_->robot_state.joints[i].pos);
            state_msg.data.push_back(robot_data_->robot_state.joints[i].vel);
        }
        
        robot_state_pub_.publish(state_msg);
        return 0;
    }

private:
    RobotData *robot_data_ = nullptr;
    ros::Publisher robot_state_pub_;
};
