/**
 * @file controller.cpp

 * @brief controllerの受取実装
**/
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
using std::abs;

geometry_msgs::Twist robot_vel;
std_msgs::Bool change_front_msg;

namespace dozap {
  inline double map(double x, double in_min, double in_max, double out_min,
      double out_max) {
    return (double)(x - in_min) * (out_max - out_min) / (in_max - in_min) +
      out_min;
  }
}
bool flag_change = false;
bool flag_change_prev = false;
void joyCallback(const sensor_msgs::Joy &msg) {
  robot_vel.linear.x = dozap::map(msg.axes[0], -1, 1,  0.5, -0.5);
  robot_vel.linear.y = dozap::map(msg.axes[1], -1, 1, -0.5,  0.5);
  //コントローラの右側のX軸のみを利用する
  robot_vel.angular.z = dozap::map(msg.axes[3], -1, 1, 0.5, -0.5);
  if(flag_change_prev == false && msg.buttons[6] == true){
    flag_change = !flag_change;
    change_front_msg.data = flag_change;
  }
  flag_change_prev = msg.buttons[6];
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "xbox");
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ros::Publisher change_front_pub = n.advertise<std_msgs::Bool>("/change_front", 10);
  ros::Subscriber controller_sub = n.subscribe("joy", 10, joyCallback);
  ros::Rate loop_rate(100);

  while (ros::ok()) {
    if(abs(robot_vel.linear.x) < 0.05) robot_vel.linear.x = 0;
    if(abs(robot_vel.linear.y) < 0.05) robot_vel.linear.y = 0;
    if(abs(robot_vel.angular.z) < 0.05) robot_vel.angular.z = 0;
    vel_pub.publish(robot_vel);
    change_front_pub.publish(change_front_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
