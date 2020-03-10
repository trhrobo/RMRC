#include "dynamixel/dynamixel.h"
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

using std::vector;

#define front_right 0
#define front_left 1
#define rear_right 2
#define rear_left 3

vector<double> angle_goal{0, 0, 0, 0};
vector<double> angle_now{0, 0, 0, 0};
//enum dynamixel_name { front_right, front_left, rear_right, rear_left };

void dynamixelCallback(const std_msgs::Float64MultiArray &msg) {
  for (int i = 0; i < msg.data.size(); ++i) {
    angle_goal[i] = msg.data[i];
  }
}

void jointCallback(const sensor_msgs::JointState &msg){
  for(int i = 0; i < 4; ++i){
    angle_now[i] = msg.position[i];
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamixel_pos");
  ros::NodeHandle n;
  ros::ServiceClient dynamixel_service = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
  ros::Subscriber servo_sub = n.subscribe("flipper_semi_autonomous", 45, dynamixelCallback);
  ros::Subscriber joint_sub = n.subscribe("/dynamixel_workbench/joint_states", 10, jointCallback);

  dynamixel servo[4] = {front_right, front_left, rear_right, rear_left};

  dynamixel_workbench_msgs::DynamixelCommand srv;
  ros::Rate loop_rate(45);

  while (ros::ok()) {

    for (int i = 0; i < 4; ++i) {
      srv.request.command = "_";
      srv.request.id = i + 1;
      srv.request.addr_name = "Goal_Position";
      srv.request.value = servo[i].dynamixelSet(angle_goal[i], angle_now[i]);
      dynamixel_service.call(srv);
    }

    ROS_INFO("angle1= %lf | angle2= %lf | angle3 = %lf |angle4 = %lf", angle_now[0], angle_now[1], angle_now[2], angle_now[3]);
    //ROS_INFO("Joint1= %lf | Joint2= %lf | Joint3 = %lf |Joint4 = %lf", angle_goal[0], angle_goal[1], angle_goal[2], angle_goal[3]);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
