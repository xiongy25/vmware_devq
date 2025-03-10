#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <string>

class RobotCommandPublisher {
private:
  ros::NodeHandle nh_;
  ros::Publisher mode_pub_;
  ros::Publisher velocity_pub_;

public:
  RobotCommandPublisher() {
    // 初始化发布者
    mode_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/robot_control/set_mode", 10);
    velocity_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/robot_control/set_velocity", 10);
    
    ROS_INFO("Robot Command Publisher initialized");
  }

  // 发送控制模式命令
  void sendModeCommand(int mode) {
    std_msgs::Int32MultiArray mode_msg;
    mode_msg.data.push_back(mode);
    
    ROS_INFO("Sending mode command: %d", mode);
    mode_pub_.publish(mode_msg);
  }

  // 发送速度控制命令
  void sendVelocityCommand(float x, float y, float yaw) {
    std_msgs::Float32MultiArray vel_msg;
    vel_msg.data.push_back(x);    // 前进/后退速度
    vel_msg.data.push_back(y);    // 左右移动速度
    vel_msg.data.push_back(yaw);  // 旋转速度
    
    ROS_INFO("Sending velocity command: [%f, %f, %f]", x, y, yaw);
    velocity_pub_.publish(vel_msg);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_command_publisher");
  
  RobotCommandPublisher publisher;
  ros::Rate loop_rate(10);  // 10 Hz
  
  // 简单的命令行参数处理示例
  if (argc > 1) {
    std::string cmd = argv[1];
    
    if (cmd == "stand") {
      publisher.sendModeCommand(2);  // 站立模式
    } else if (cmd == "liedown") {
      publisher.sendModeCommand(1);  // 趴下模式
    } else if (cmd == "forward") {
      publisher.sendVelocityCommand(0.1, 0.0, 0.0);  // 前进
    } else if (cmd == "backward") {
      publisher.sendVelocityCommand(-0.1, 0.0, 0.0);  // 后退
    } else if (cmd == "turn_left") {
      publisher.sendVelocityCommand(0.0, 0.0, 0.3);  // 左转
    } else if (cmd == "turn_right") {
      publisher.sendVelocityCommand(0.0, 0.0, -0.3);  // 右转
    } else {
      ROS_ERROR("Unknown command: %s", cmd.c_str());
      return 1;
    }
    
    // 确保消息发送出去
    ros::spinOnce();
    loop_rate.sleep();
    
    return 0;
  }
  
  ROS_INFO("No command specified. Exiting.");
  return 0;
} 