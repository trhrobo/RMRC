/**
 * @file flipper_semi_autonomous.cpp

 * @brief flipper制御の実装
**/
#include <cmath>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <array>
#include <tuple>
#include "robot_motion/semi_autonomous.h"
#include "dynamixel/dynamixel.h"
#include "dynamixel.cpp"
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

using std::array;
using std::tuple;

#define DEBUG 1
#define front_right 0
#define front_left 1
#define rear_right 2
#define rear_left 3

constexpr double flipper_m = 100;
constexpr double flipper_lg = 100;
constexpr double gravity = 9.81;
constexpr double original_theta = 90.0;
constexpr double autonomous_max_theta = 75.0;
constexpr double autonomous_min_theta = -90.0;
constexpr int MAX_POSITION_VALUE = 1048575;
constexpr int MIN_POSITION_VALUE = -1048575;
constexpr bool TORQUE_ENABLE = 1;
constexpr bool TORQUE_DISABLE = 0;
array<double, 4> current_dynamixel_pos{};
array<double, 4> current_dynamixel_theta{};
array<double, 4> current_dynamixel_torque{};
array<double, 4> theta_ref{};
array<double, 4> pos_ref{};

template<typename T>
inline T degToRad(const T deg){
  return deg * (M_PI / 180.0);
}
template<typename T>
inline T radToDeg(const T rad){
  return rad * (180.0 / M_PI);
}

enum class keyFlag{
  NOMAL = 0,
  ALL,
  AUTO
};

//-----------------------------------------------------------------
namespace safetyCheck{
  //過負荷の確認
  void torque_limit{
    try{

    }catch{

    }
  }
}

namespace all{
  //using calc_f = void(*)();
  //関数ポインタの使い方を間違えている
  std::function<void(void)> calc_f;
  explicit void setRotation(const calc_f Func){
    Func();
  }
  void reset() {for (int i = 0; i < dynamixel_num.size(); ++i) {theta_ref[i] = original_theta;}}
  void forward() {
    theta_ref[0] = MAX_POSITION_VALUE;
    for (int i = 1; i < dynamixel_num.size(); ++i) {theta_ref[i] = theta_ref[0];}
  }
  void reverse() {
    theta_ref[0] = MIN_POSITION_VALUE;
    for (int i = 1; i < dynamixel_num.size(); ++i) {theta_ref[i] = theta_ref[0];}
  }
  void nomal(){
    for (int i = 1; i < dynamixel_num.size(); ++i) {theta_ref[i] = theta_ref[0];}
  }
}
//-----------------------------------------------------------------

namespace nomal{
  //using calc_f = void(*)(int);
  //関数ポインタの使い方を間違えている
  std::function<void(void)> calc_f;
  explicit void setRotation(const int id, const calc_f Func){
    Func(id);
  }
  void forward(const int id){
    pos_ref[id] = MAX_POSITION_VALUE;
  }
  void reverse(const int id){
    pos_ref[id] = MIN_POSITION_VALUE;
  }
  void nomal(const int id){
    pos_ref[id] = current_dynamixel_pos;
  }
}

namespace gyroControl{
  enum class Lean{
    forward,
    backward,
    nomal
  }
  Lean gyroFeedback(){
    //ラジアンと角度を全体で管理する
    constexpr double threshold_forward_z = 330;
    constexpr double threshold_backward_z = 30;
    Lean leanState;
    auto leanCheck = [&](const double angle_z) -> Lean{
      Lean leanNow;
      if(angle_z > threshold_forward_z){
        leanState = Lean::forward;
      }else if(angle_z < threshold_backward_z){
        leanState = Lean::backward;
      }else{
        leanState = Lean::nomal;
      }
      return leanState; 
    }
    swith(leanCheck(gyro_robot.z)){
      case Lean::forward:
        //前傾姿勢の場合はコンプライアンス制御をする
        break;
      case Lean::backward:
        //後傾姿勢の場合は通常の姿勢制御をする
        break;
      default:
        //通常姿勢の場合は通常の姿勢制御をする
        break;
    }
  }
}

constexpr double Kp = 1.0;
constexpr double Kd = 1.0;
namespace DynamixelControl{
  template<typename T>
  T TorqueControl(T theta_d){
    //トルク制御の実装
    //重力補償項の追加
    T gravity_compensation = flipper_m * gravity * cos(degToRad(theta_now));
    return radToDeg<double>(Kp * (degToRad<double>(theta_d) - degToRad<double>(theta_now)) - Kd * (angular) + gravity_compensation);
  }
  template<typename T>
  T PosControl(T theta_d){

  }
}
//現在角度とトルクを取得
//dynamixelIDとjointstateの順番が違う
constexpr array<int, 4> dynamixel_num{0, 1, 3, 2};
void jointStateCallback(const sensor_msgs::JointState &jointstate) {
  for(int i = 0; i < dynamixel_num.size(); ++i){
    current_dynamixel_pos[i] = jointstate.position[dynamixel_num[i]];
    current_dynamixel_theta[i] = degToRad<double>((jointstate.position[dynamixel_num[i]] + M_PI));
    current_dynamixel_torque[i] = jointstate.effort[dynamixel_num[i]];
  }
}

template<typename T>
struct Gyro{
  T x;
  T y;
  T z;
}

Gyro<double> gyro_robot; 

//ロボットの現在角度を取得
//rosのtfに合わせてあとで型を変える
namespace RobotState{
void gyroCallback(const std_msgs::Float64 &msg) {
  gyro_robot.x = msg.data.x;
  gyro_robot.y = msg.data.y;
  gyro_robot.z = msg.data.z;
}
void stateManagement(){
  //ロボットの前後を入れ替える
  if(front_flag == true){
    //配列番号を変える
    std::swap(dynamixel_num[0], dynamixel_num[2]);
    std::swap(dynamixel_num[1], dynamixel_num[3]);
  }
}
}

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

//NodeHandleとかもグローバル宣言にしたほうがいいかも
ros::ServiceClient dynamixel_service;
bool serviceCallPos(dynamixel &servo){
  for(int i = 0; i < dynamixel_num.size(); ++i){
    srv.request.command = "_";
    srv.request.id = i + 1;
    srv.request.addr_name = "Goal_Position";
    srv.request.value = pos_ref[i];
    dynamixel_service.call(srv);
  }
  //いるかあとで考える
  ros::spinOnce();
  return 
}

bool serviceCallTheta(dynamixel &servo){
  for (int i = 0; i < dynamixel_num.size(); ++i) {
    if(theta_ref[i] > 360)[i] -= 360;
    if(theta_ref[i] > 0)theta_ref[i] += 360;
    ROS_INFO("theta_ref[%d] %lf current_dynamixel_theta[%d] %lf", i, theta_ref[i], i, current_dynamixel_theta[i]);
    srv.request.command = "_";
    srv.request.id = i + 1;
    srv.request.addr_name = "Goal_Position";
    srv.request.value = servo[i].dynamixelSet(theta_ref[i], current_dynamixel_theta[i]);
    dynamixel_service.call(srv);
  }
  ros::spinOnce();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "semi_autonomous");
  ros::NodeHandle n;

  dynamixel_service = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
  ros::Subscriber feedback_sub = n.subscribe("/dynamixel_workbench/joint_states", 10, jointStateCallback);
  ros::Subscriber gyro_sub = n.subscribe("gyro", 10, gyroCallback);
  ros::Subscriber controller_sub = n.subscribe("joy", 10, joyCallback);
  ros::Rate loop_rate(400);
  dynamixel<double> servo[4] = {front_right, front_left, rear_right, rear_left};
  semiAuto robot_model(n);

  dynamixel_workbench_msgs::DynamixelCommand srv;
  while (ros::ok()) {
    for(int i = 0; i < dynamixel_num.size(); ++i){
      if(current_dynamixel_theta[i] <= 0)current_dynamixel_theta[i] = 360 + current_dynamixel_theta[i];
    }
    switch(controller_state){
      case keyFlag::ALL:
        if ((controller_key[0] < 0) or (controller_key[1] < 0)){
          all::setRotation(all::forward);
        }else if((controller_key[2] == true) or (controller_key[3] == true)){
          all::setRotation(all::reverse);
        }else if(flag_reset){
          all::reset();
        }else{
          all::nomal();
        }
        serviceCallPos(servo);
        break;

      case keyFlag::AUTO:
        robot_model.main(theta_ref);
        serviceCallTheta(servo);
        break;

      default:
        for(int i = 0; i < dynamixel_num.size(); ++i){
          if(controller_key[i] < 0){
            buttons_reverse == 1 ? nomal::setRotation(i, nomal::reverse) : nomal::setRotation(i, nomal::forward);
          }else{
            if((i == 2 or i == 3) && controller_key[i] == true){
              buttons_reverse == 1 ? nomal::setRotation(i, nomal::reverse) : nomal::setRotation(i, nomal::forward);
            }
          }
        }
        serviceCallPos(servo);
        break;
    }
    loop_rate.sleep();
  }
}
