#include <iostream>
#include <mutex>
#include <pigpiod_if2.h>
#include <queue>
#include <ros/ros.h>
#include <thread>
#include <vector>

using std::vector;

int main(int argc, char argv) {
  ros::init(argc, argv, "serial");
