#include <cmath>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <array>
#include "robot_motion/semi_autonomous.h"
#include "dynamixel/dynamixel.h"
#include "dynamixel.cpp"
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

using std::array;

#define DEBUG 1
#define front_right 0
#define front_left 1
#define rear_right 2
#define rear_left 3

constexpr double original_theta = 90.0;
constexpr double autonomous_max_theta = 75.0;
constexpr double autonomous_min_theta = -90.0;
array<double, 4> current_dynamixel_theta{};
array<double, 4> current_dynamixel_torque{};
double theta_ref[4]{0, 0, 0, 0};
double gyro_robot{};

enum class keyFlag{
  NOMAL = 0,
  ALL,
  AUTO
};

//-----------------------------------------------------------------

namespace all {
  using calc_f = void(*)();
  bool check() {
    for (int i = 0; i < 4; ++i) {
      if ((current_dynamixel_theta[i] + 1 > theta_ref[i]) and (current_dynamixel_theta[i] - 1 < theta_ref[i])) {
      } else {
        return false;
      }
    }
    return true;
  }
  void setRotation(calc_f func){
    func();
  }
  void reset() {for (int i = 0; i < 4; ++i) {theta_ref[i] = original_theta;}}
  void forward() {
    if (all::check()) theta_ref[0] += 1.3;
    for (int i = 1; i < 4; ++i) {theta_ref[i] = theta_ref[0];}
  }
  void reverse() {
    if (all::check()) theta_ref[0] -= 1.3;
    for (int i = 1; i < 4; ++i) {theta_ref[i] = theta_ref[0];}
  }
}
//-----------------------------------------------------------------

namespace nomal{
  using calc_f = void(*)(int);
  void setRotation(int id, calc_f func){
    func(id);
  }
  void forward(int id){
    if ((current_dynamixel_theta[id] + 1 > theta_ref[id]) and (current_dynamixel_theta[id] - 1 < theta_ref[id])) {
      theta_ref[id] += 10;
    }else if ((current_dynamixel_theta[id] + 1 > theta_ref[id]) and (current_dynamixel_theta[id] > 355.5)) {
      theta_ref[id] += 10;
    }
  }
  void reverse(int id){
    if ((current_dynamixel_theta[id] + 1 > theta_ref[id]) and (current_dynamixel_theta[id] - 1 < theta_ref[id])) {
      theta_ref[id] -= 10;
    }else if ((current_dynamixel_theta[id] + 1 > theta_ref[id]) and (current_dynamixel_theta[id] > 355.5)) {
      theta_ref[id] -= 10;
    }
  }
}
//現在角度とトルクを取得
void jointStateCallback(const sensor_msgs::JointState &jointstate) {
  current_dynamixel_theta[0] = (jointstate.position[1] + M_PI) * 180 / M_PI;
  current_dynamixel_torque[0] = jointstate.effort[1];

  current_dynamixel_theta[1] = (jointstate.position[0] + M_PI) * 180 / M_PI;
  current_dynamixel_torque[1] = jointstate.effort[0];

  current_dynamixel_theta[2] = (jointstate.position[3] + M_PI) * 180 / M_PI;
  current_dynamixel_torque[2] = jointstate.effort[3];

  current_dynamixel_theta[3] = (jointstate.position[2] + M_PI) * 180 / M_PI;
  current_dynamixel_torque[3] = jointstate.effort[2];
}

//ロボットの現在角度を取得
void gyroCallback(const std_msgs::Float64 &msg) { gyro_robot = msg.data; }

int flag_key{};
bool buttons_reverse = false;
bool flag_reset = false;
bool prev_reset = false;
array<double, 4> controller_key{};

keyFlag controller_state;

//コントローラ値を入力
void joyCallback(const sensor_msgs::Joy &controller) {
  buttons_reverse = controller.buttons[2];
  controller_key[0] = controller.axes[5];
  controller_key[1] = controller.axes[2];
  controller_key[2] = controller.buttons[5];
  controller_key[3] = controller.buttons[4];
  if(controller.buttons[3] == true) controller_state = keyFlag::ALL;
  if(controller.buttons[8] == true) controller_state = keyFlag::AUTO;
  if(controller.buttons[1] == true) controller_state = keyFlag::NOMAL;
  // Bキーで全てのフリッパーの角度を90°
  if ((prev_reset == false) and controller.buttons[1] == true) flag_reset = !flag_reset;
  prev_reset = controller.buttons[1];
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "semi_autonomous");
  ros::NodeHandle n;

  ros::ServiceClient dynamixel_service = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
  ros::Subscriber feedback_sub = n.subscribe("/dynamixel_workbench/joint_states", 10, jointStateCallback);
  ros::Subscriber gyro_sub = n.subscribe("gyro", 10, gyroCallback);
  ros::Subscriber controller_sub = n.subscribe("joy", 10, joyCallback);
  ros::Rate loop_rate(400);
  dynamixel<double> servo[4] = {front_right, front_left, rear_right, rear_left};
  semiAutonomous robot_model(n);

  dynamixel_workbench_msgs::DynamixelCommand srv;
  while (ros::ok()) {
    for(int i = 0; i < 4; ++i){
      if(current_dynamixel_theta[i] <= 0)current_dynamixel_theta[i] = 360 + current_dynamixel_theta[i];
    }
    switch(controller_state){
      case keyFlag::ALL:
        if ((controller_key[0] < 0) or (controller_key[1] < 0)) all::setRotation(all::forward);
        if ((controller_key[2] == true) or (controller_key[3] == true)) all::setRotation(all::reverse);
        if (flag_reset) all::reset();
        break;
      case keyFlag::AUTO:
        robot_model.main(theta_ref);
        break;
      default:
        for(int i = 0; i < 4; ++i){
          if(controller_key[i] < 0){
            buttons_reverse == 1 ? nomal::setRotation(i, nomal::reverse) : nomal::setRotation(i, nomal::forward);
          }else{
            if((i == 2 or i == 3) && controller_key[i] == true){
              buttons_reverse == 1 ? nomal::setRotation(i, nomal::reverse) : nomal::setRotation(i, nomal::forward);
            }
          }
        }
    }
    for (int i = 0; i < 4; ++i) {
      if(theta_ref[i] > 360)theta_ref[i] -= 360;
      if(theta_ref[i] > 0)theta_ref[i] += 360;
      ROS_INFO("theta_ref[%d] %lf current_dynamixel_theta[%d] %lf", i, theta_ref[i], i, current_dynamixel_theta[i]);
      srv.request.command = "_";
      srv.request.id = i + 1;
      srv.request.addr_name = "Goal_Position";
      srv.request.value = servo[i].dynamixelSet(theta_ref[i], current_dynamixel_theta[i]);
      dynamixel_service.call(srv);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
