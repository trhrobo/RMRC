#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "gui_robot");
  ros::NodeHandle n;
  ros::Subscriber dynamixel_sub =
      n.subscribe("/dynamixel/joint_state", 10, dynamixelCallback);
