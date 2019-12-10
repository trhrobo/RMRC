#include <cmath>
#include <comprehensive/Button.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>

using std::vector;

vector<double> angle_dynamixel = {0, 0, 0, 0};
double distance_front = 0, distance_back = 0, angle_robot = 0;
bool flag_manual;

enum class Status { MANUAL; UP_BOTH; UP_EITHER; };

void angleCallback(const std_msgs::Float64MultiArray &msg) {
  for (auto &num : angle_dynamixel) {
    num = msg.data[num];
  }
}

void distanceCallback(const std_msgs::Float64MultiArray &msg) {
  distance_front = msg.data[0];
  distance_back = msg.data[1];
}

void gyroCallback(const std_msgs::Float64MultiArray &msg) {
  angle_robot = msg.data;
}

void controlleCallback(const comprehensive::Button &msg) {
  msg.dynamixel_right_front
      ? flag_manual = true
      : msg.dynamixel_right_back
            ? flag_manual = true
            : msg.dynamixel_left_front
                  ? flag_manual = true
                  : msg.dynamixel_left_back ? flag_manual = true : flag_manual =
                                                                       false;
}
bool judge_angle_front(double front, double distance_front);
bool judge_angle_back(double robot, double back, double distance_back);
bool flag_down_back(double robot, bool contacts_front);

Status judge_angle(double &front, double &back, double &distance_front,
                   double &distance_back, double &robot_angle);

int main(int argc, char **argv) {
  ros::init(argc, argv, "flipper_angle");
  ros::NodeHandle n;
  // send message type is radian
  ros::Publisher angle_pub =
      n.advertise<std_msgs::Float64MultiArray>("flipper_angle", 10);
  ros::Subscriber servo_angle_sub = n.subscribe<"angle_now", 10, angleCallback>;
  ros::Subscriber distance_sub = n.subscribe<"distance", 10, distanceCallback>;
  ros::Subscriber gyro_sub = n.subscribe<"gyro_pitch", 10, gyroCallback>;
  ros::Subscriber xbox_sub = n.subscribe<"xbox", 10, controllerCallback>;

  ros::Rate loop_rate(100);

  std_msgs::Float64MultiArray angle_data;
  angle_data.resize(4);
  vector<double> dynamixel_goal_angle{0, 0, 0, 0};
  double angle_front = 0, angle_back = 0;

  // setting flipper_parameter
  constexpr double flipper_length = 100;
  constexpr double nomal_angle_front = 25;
  constexpr double nomal_angle_back = 25;
  constexpr double wall_angle_front = 10;
  constexpr double wall_angle_back = 0;

  Status servoStatus;

  while (ros::ok()) {
    if (flag_manual == true)
      servoStatus = MANUAL;
    else
      servoStatus = judge_angle(double &angle_front, double &angle_back,
                                double &distance_front, double &distance_back,
                                double &robot_angle);
    switch (servoStatus) {
    case MANUAL:
      break;
    case UP_BOTH:
      break;
    case UP_EITHER:
      break;
    }
    for (auto &num : dynamixel_goal_angle) {
      angle_data.data[num] = angle[num];
    }
    angle_pub.publish(angle_data);
    ros::spinOnce;
    loop_rate.sleep();
  }
}

Status judge_angle(double &front, double &back, double &distance_front,
                   double &distance_back, double &robot_angle) {
  bool front_contacts = false, back_contacts = false;
  if (distance_angle < flipper_length) {
    if ()
      front_contacts = true;
    else
      front_contacts = false;
  } else if () {
    // if angle > 0 the robot cannot down the flipper
    robot_angle > 0 ? back_contacts = true : back_contacts = false;
  }
  (front_contacts == true && back_contacts == true)
      ? return UP_BOTH
      : (front_contacts == false && back_contacts true) ? return UP_EITHER
                                                        : return MANUAL;
}
