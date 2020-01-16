#include <cmath>
#include <comprehensive/Button.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <vector>

using std::vector;
using std::abs;

std_msgs::Float64MultiArray msg;
msg.data.resize(2);

double speed = 0;
double angle = 0;
vector<double> wheel{0, 0};
vector<double> pwm{0, 0};

//適切な速度にする必要あり
//現在はPWM値として255としているが[m/sec]にする必要あり
/*void joyCallback(const sensor_msgs::Joy &controller) {
  speed = hypot(controller.axes[1], controller.axes[0]) * speed_gain;
  // wheel.push_back(abs(-1 - controller.axes[0]));
  // wheel.push_back(abs(1 - controller.axes[1]));
  wheel[0] = (-1.0 - controller.axes[0]);
  wheel[1] = (1.0 - controller.axes[1]);
  wheel[0] = abs(wheel[0]);
  wheel[1] = abs(wheel[1]);
  // ROS_INFO("%lf", controller.axes[0]);
  wheel[0] > 1 ? wheel[0] = 1 : wheel[0] = wheel[0];
  wheel[1] > 1 ? wheel[1] = 1 : wheel[1] = wheel[1];
  pwm[0] = wheel[0] * speed;
  pwm[1] = wheel[1] * speed;
  for (auto &t : pwm)
    controller.axes[1] >= 0 ? t = t : t = -t;
  ROS_INFO("pwm[0] = %d : pwm[1] = %d", pwm[0], pwm[1]);
}*/

void velCallback(const geometry_msgs::Twist &vel) {
  speed = hypot(vel.linear.y, vel.linear.x);
  angle = atan2(vel.linear.x, vel.linear.y);
  msg.data[0] = sin(angle + (M_PI / 4)) * speed;
  msg.data[1] = sin(angle - (M_PI / 4)) * speed;
  if(vel.angular.z > 0){
    msg.data[0] = vel.angular.z;
    msg.data[1] = -vel.angular.z;
  }
}

double turn_right, turn_left;

int main(int argc, char **argv) {
  ros::init(argc, argv, "wheel_control");
  ros::NodeHandle n;
  ros::Publisher wheel_pub =
      n.advertise<std_msgs::Float64MultiArray>("wheel", 10);
  ros::Subscriber controller_sub = n.subscribe("cmd_vel", 10, velCallback);
  ros::Rate loop_rate(1000);
  msg.data.resize(2);

  while (ros::ok()) {
    wheel_pub.publish(msg);
    loop_rate.sleep();
    ros::spinOnce();
  }
}
