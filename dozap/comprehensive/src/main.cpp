#include <cmath>
#include <comprehensive/Button.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16MultiArray.h>
#include <vector>

using std::vector;
using std::abs;

std_msgs::Int16MultiArray msg;

int speed = 0;
// vector<int> wheel;
// vector<int> pwm;
double wheel[2] = {};
int pwm[2] = {};
void joyCallback(const sensor_msgs::Joy &controller) {
  speed = hypot(controller.axes[1], controller.axes[0]) * 255;
  abs(speed) > 255 ? speed = 255 : speed = speed;
  // wheel.push_back(abs(-1 - controller.axes[0]));
  // wheel.push_back(abs(1 - controller.axes[1]));
  wheel[0] = (-1.0 - controller.axes[0]);
  wheel[1] = (1.0 - controller.axes[0]);
  wheel[0] = abs(wheel[0]);
  wheel[1] = abs(wheel[1]);
  // ROS_INFO("%lf", controller.axes[0]);
  wheel[0] > 1 ? wheel[0] = 1 : wheel[0] = wheel[0];
  wheel[1] > 1 ? wheel[1] = 1 : wheel[1] = wheel[1];
  pwm[0] = (int)(wheel[0] * speed);
  pwm[1] = (int)(wheel[1] * speed);
  for (auto &t : pwm)
    controller.axes[1] >= 0 ? t = t : t = -t;
  ROS_INFO("pwm[0] = %d : pwm[1] = %d", pwm[0], pwm[1]);
}
int turn_right, turn_left;
void controllerCallback(const comprehensive::Button msg) {
  turn_right = msg.rotation_right;
  turn_left = msg.rotation_left;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "wheel_control");
  ros::NodeHandle n;
  ros::Publisher wheel_pub =
      n.advertise<std_msgs::Int16MultiArray>("wheel", 10);
  ros::Subscriber controller_sub = n.subscribe("joy", 10, joyCallback);
  ros::Subscriber xbox_sub = n.subscribe("xbox", 10, controllerCallback);
  ros::Rate loop_rate(1000);
  msg.data.resize(2);

  while (ros::ok()) {
    for (int i = 0; i < 2; ++i) {
      msg.data[i] = pwm[i];
    }
    turn_right != 0 ? msg.data[0] = -turn_right : msg.data[1] = turn_left;
    turn_left != 0 ? msg.data[0] = turn_left : msg.data[1] = -turn_left;
    wheel_pub.publish(msg);
    loop_rate.sleep();
    ros::spinOnce();
  }
}
