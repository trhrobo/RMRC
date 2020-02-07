#include "encoder/encoder.h"
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

namespace right {
constexpr int pin_A = 17;
constexpr int pin_B = 27;
constexpr int RANGE = 500;
constexpr int MULTIPLICATION = 1;
constexpr double DIAMETER = 100.0;
}

namespace left {
constexpr int pin_A = 23;
constexpr int pin_B = 24;
constexpr int RANGE = 512;
constexpr int MULTIPLICATION = 1;
constexpr double DIAMETER = 100.0;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "encoder");
  ros::NodeHandle n;
  ros::Publisher encoder_pub =
      n.advertise<std_msgs::Float64MultiArray>("encoder", 10);
  std_msgs::Float64MultiArray msg;
  msg.data.resize(2);
  AMT encoder_right((int)right::pin_A, (int)right::pin_B, 1);
  AMT encoder_left((int)left::pin_A, (int)left::pin_B, 1);
  ros::Rate loop_rate(500);
  int now_pulse_right = 0;
  int prev_pulse_right = 0;
  int now_pulse_left = 0;
  int prev_pulse_left = 0;
  while (ros::ok()) {
    now_pulse_right = encoder_right.get();
    now_pulse_left = encoder_left.get();
    ROS_INFO("right = %d, left = %d", now_pulse_right, now_pulse_left);
    msg.data[0] = ((((static_cast<double>(now_pulse_right) - static_cast<double>(prev_pulse_right)) /
                     (right::RANGE * right::MULTIPLICATION))) *
                   right::DIAMETER) / 500;
    msg.data[1] = ((((static_cast<double>(now_pulse_left) - static_cast<double>(prev_pulse_left)) /
                     (left::RANGE * left::MULTIPLICATION))) *
                   left::DIAMETER) / 500;
    encoder_pub.publish(msg);
    prev_pulse_right = now_pulse_right;
    prev_pulse_left = now_pulse_left;
    ros::spinOnce();
    loop_rate.sleep();
  }
}
