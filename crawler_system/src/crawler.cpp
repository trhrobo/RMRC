/**
 * @file main.cpp

 * @brief main実装
**/
#include <iostream>
#include <cmath>
#include <utility>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

using std::vector;
using std::abs;
using std::swap;

double speed = 0;
double angle = 0;
vector<double> wheel_speed{0, 0};
float gyro_data{};

void velCallback(const geometry_msgs::Twist &vel) {
  speed = hypot(vel.linear.y, vel.linear.x);
  angle = atan2(vel.linear.x, vel.linear.y);
}
void gyroCallback(const std_msgs::Float32MultiArray &msg){
  gyro_data = msg.data;
}
bool change_front;
void frontCallback(const std_msgs::Bool &msg){
  change_front = msg.data;
}
void invKinematics(){
  wheel_speed[0] = sin(angle + (M_PI / 4)) * speed;
  wheel_speed[1] = sin(angle - (M_PI / 4)) * speed;
  if(change_front){
    swap(wheel_speed[0], wheel_speed[1]);
    for(int i = 0; i < 2; ++i){
      wheel_speed[i] = -wheel_speed[i];
    }
  }
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "wheel_control");
  ros::NodeHandle n;
  ros::Publisher wheel_pub = n.advertise<std_msgs::Float32MultiArray>("/motor_speed", 10);
  ros::Subscriber controller_sub = n.subscribe("/cmd_vel", 10, velCallback);
  
  ros::Subscriber gyro_sub = n.subscribe("gyro", 10, gyroCallback);
  ros::Subscriber change_front_sub = n.subscribe("/change_front", 10, frontCallback);
  ros::Rate loop_rate(1000);

  std_msgs::Float32MultiArray msg;
  msg.data.resize(2);

  while (ros::ok()) {
    invKinematics();
    for (int i = 0; i < 2; ++i) {
      msg.data[i] = wheel_speed[i];
    }
    wheel_pub.publish(msg);
    loop_rate.sleep();
    ros::spinOnce();
  }
}
