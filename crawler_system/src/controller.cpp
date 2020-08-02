#include <cmath>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

using std::abs;

geometry_msgs::Twist robot_vel;

namespace dozap {
  inline double map(double x, double in_min, double in_max, double out_min,
      double out_max) {
    return (double)(x - in_min) * (out_max - out_min) / (in_max - in_min) +
      out_min;
  }
}
void joyCallback(const sensor_msgs::Joy &msg) {
  robot_vel.linear.x = dozap::map(msg.axes[0], -1, 1, 0.5, -0.5);
  robot_vel.linear.y = dozap::map(msg.axes[1], -1, 1, -0.5, 0.5);
  //コントローラの右側のX軸のみを利用する
  robot_vel.angular.z = dozap::map(msg.axes[3], -1, 1, 0.5, -0.5);
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "xbox");
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ros::Subscriber controller_sub = n.subscribe("joy", 10, joyCallback);
  ros::Rate loop_rate(100);

  while (ros::ok()) {
    if(abs(robot_vel.linear.x) < 0.05) robot_vel.linear.x = 0;
    if(abs(robot_vel.linear.y) < 0.05) robot_vel.linear.y = 0;
    if(abs(robot_vel.angular.z) < 0.05) robot_vel.angular.z = 0;
    vel_pub.publish(robot_vel);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
