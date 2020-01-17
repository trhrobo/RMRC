#include <cmath>
#include <comprehensive/Button.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>

using std::vector;

vector<double> dynamixel_current_angle = {0, 0, 0, 0};
vector<double> dynamixel_goal_angle = {0, 0, 0, 0};
double distance_front = 0, distance_back = 0, gyro_robot = 0,
       prev_gyro_robot = 0;
bool flag_manual = false;
bool flag_prev_status = false;
// judgement of rotate the rear flipper in the negative direction
bool flag_lower = false;

enum Status { MANUAL, UP_BOTH, UP_EITHER };

void angleCallback(const std_msgs::Float64MultiArray &msg) {
  for (auto &num : dynamixel_current_angle) {
    num = msg.data[num];
  }
}

void distanceCallback(const std_msgs::Float64MultiArray &msg) {
  distance_front = msg.data[0];
  distance_back = msg.data[1];
}

void gyroCallback(const std_msgs::Float64 &msg) { gyro_robot = msg.data; }

void controlleCallback(const comprehensive::Button &msg) {
  msg.dynamixel_right_front
      ? flag_manual = true
      : msg.dynamixel_right_back
            ? flag_manual = true
            : msg.dynamixel_left_front
                  ? flag_manual = true
                  : msg.dynamixel_left_back ? flag_manual = true : flag_manual =
                                                                       false;
  dynamixel_goal_angle[0] = msg.dynamixel_right_front;
  dynamixel_goal_angle[1] = msg.dynamixel_left_front;
  dynamixel_goal_angle[2] = msg.dynamixel_right_back;
  dynamixel_goal_angle[3] = msg.dynamixel_left_back;
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
    else {
      servoStatus = judge_angle(double &angle_front, double &angle_back,
                                double &distance_front, double &distance_back,
                                double &robot_angle);
      //----------calcuration of angle_front----------
      if (dynamixel_current_angle[0] > 0 && dynamixel_current_angle[1] < 90) {
        if (dynamixel_current_angle[1] > 0 && dynamixel_current_angle[2] < 90) {
          angle_front =
              (dynamixel_current_angle[0] + dynamixel_current_angle[1]) / 2;
        } else {
          angle_front = dynamixel_current_angle[0];
        }
      } else if (dynamixel_current_angle[1] > 0 &&
                 dynamixel_current_angle[2] < 90) {
        angle_front = dynamixel_current_angle[1];
      } else {
        angle_front = nomal_angle_front;
      }
      //----------calcuration of angle_back----------
      if (dynamixel_current_angle[2] > 0 && dynamixel_current_angle[2] < 90) {
        if (dynamixel_current_angle[3] > 0 && dynamixel_current_angle[3] < 90) {
          angle_back =
              (dynamixel_current_angle[2] + dynamixel_current_angle[3]) / 2;
        } else {
          angle_back = dynamixel_current_angle[2];
        }
      } else if (dynamixel_current_angle[3] > 0 &&
                 dynamixel_current_angle[3] < 90) {
        angle_back = dynamixel_current_angle[3];
      } else {
        angle_back = nomal_angle_back;
      }
    }
    //----------calcuration of dynamixel_goal_angle----------
    switch (servoStatus) {
    case MANUAL:
      flag_prev_status = false;
      break;
    case UP_BOTH:
      dynamixel_goal_angle[0] = 15;
      dynamixel_goal_angle[1] = 15;
      dynamixel_goal_angle[2] = 0;
      dynamixel_goal_angle[3] = 0;
      flag_prev_status = false;
      break;
    case UP_EITHER:
      dynamixel_goal_angle[0] = 25;
      dynamixel_goal_angle[1] = 25;
      if (flag_lower == false) {
        dynamixel_goal_angle[2] -= 2;
        dynamixel_goal_angle[3] -= 2;
      }
      flag_prev_status = true;
      break;
    }
    //----------set dynamixel_parameters----------
    for (auto &num : dynamixel_goal_angle) {
      angle_data.data[num] = num;
      angle_data.data[num] = (angle_data.data[num] / 180) * M_PI;
    }
    angle_pub.publish(angle_data);
    flag_manual = false;
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
  if (flag_prev_status) {
    if (prev_gyro_robot > gyro_robot - 5 && prev_gyro_robot < gyro_robot + 5) {
      flag_lower = true;
    } else {
      flag_lower = false;
    }
  } else {
    flag_lower = false;
  }
  (front_contacts == true && back_contacts == true)
      ? return UP_BOTH
      : (front_contacts == false && back_contacts true) ? return UP_EITHER
                                                        : return MANUAL;
}
