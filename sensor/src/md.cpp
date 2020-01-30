#include "vnh5019/vnh5019.h"
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int64MultiArray.h>
using std::abs;

int pwm_get[2]{};
void pwmCallback(const std_msgs::Int64MultiArray &msg) {
  pwm_get[0] = msg.data[0];
  pwm_get[1] = msg.data[1];
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vnh5019");
  ros::NodeHandle n;
  ros::Subscriber pwm_sub = n.subscribe("wheel_pwm", 10, pwmCallback);
  ros::Rate loop_rate(500);

  mdPin rightPin;
  rightPin.pin_pwm = 1;
  rightPin.pin_change_a = 2;
  rightPin.pin_change_b = 3;

  mdPin leftPin;
  leftPin.pin_pwm = 4;
  leftPin.pin_change_a = 5;
  leftPin.pin_change_b = 6;

  VNH5019 motor_right(rightPin);
  VNH5019 motor_left(leftPin);

  while (ros::ok) {
    motor_right.set(pwm_get[0]);
    motor_left.set(pwm_get[1]);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
