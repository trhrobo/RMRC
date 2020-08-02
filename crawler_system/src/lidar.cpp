#include <iostream>
#include <ros/ros.h>

void lidarCallback(const sensor_msgs) int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar");
  ros::NodeHandle n;
  ros::Subscriber lidar_sub = n.subscribe("/scan", 10, lidarCallback);
