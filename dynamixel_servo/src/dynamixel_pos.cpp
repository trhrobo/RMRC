#include "dynamixel/dynamixel.h"
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs::JointState.h>
#include <vector>

using std::vector;

#define front_right 0
#define front_left 1
#define rear_right 2
#define rear_left 3

vector<double> angle_goal{0, 0, 0, 0};
vector<double> angle_now{0, 0, 0, 0};
enum dynamixel_name { front_right, front_left, rear_right, rear_left };

void dynamixelCallback(const std_msgs::Float64MultiArray &msg) {
  for (int i = 0; i < msg.data.size(); ++i) {
    angle_goal[i] = msg.data[i];
  }
}

void jointCallback(const sensor_msgs::jointCallback &msg){
  for(int i = 0; i < 4; ++i){
    angle_now[i] = msg.position[i];
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamixel_pos");
  ros::NodeHandle n;

  ros::Publisher servo_pub = n.advertise<trajectory_msgs::JointTrajectory>(
      "/dynamixel_workbench/joint_trajectory", 10);

  // ros::Subscriber servo_sub = n.subscribe("flipper", 45, dynamixelCallback);
  ros::Subscriber servo_sub = n.subscribe("flipper_semi_autonomous", 45, dynamixelCallback);
  ros::Subscriber joint_sub = n.subscribe("/dynamixel_workbench/joint_states", 10, jointCallback);

  trajectory_msgs::JointTrajectory jtp0;

  jtp0.header.frame_id = "base_link";

  jtp0.joint_names.resize(4);

  jtp0.points.resize(1);

  //  jtp0.points[0].positions.resize(4);
  jtp0.points[0].positions.resize(4);

  jtp0.joint_names[0] = "front_right";
  jtp0.joint_names[1] = "front_left";
  jtp0.joint_names[2] = "rear_right";
  jtp0.joint_names[3] = "rear_left";

  /*
     dynamixel servo[4] = {
     {front_right},
     {front_left},
     {back_right},
     {back_left}
     };*/

  dynamixel servo[4] = {front_right, front_left, rear_right, rear_left};

  ros::Rate loop_rate(45);

  while (ros::ok()) {

    for (int i = 0; i < 4; ++i) {
      jtp0.points[0].positions[i] = servo[i].dynamixelSet(angle_goal[i], angle_now[i]);
    }

    jtp0.points[0].time_from_start = ros::Duration(0.02);
    servo_pub.publish(jtp0);
    ROS_INFO("Joint1= %lf | Joint2= %lf | Joint3 = %lf |Joint4 = %lf",
        angle_goal[0], angle_goal[1], angle_goal[2], angle_goal[3]);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
