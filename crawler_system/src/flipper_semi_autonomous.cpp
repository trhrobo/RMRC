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
#include <vector>
#include <memory>
#include "robot_motion/flipper_util.h"
#include "robot_motion/Constant.h"
#include "robot_motion/semi_autonomous.h"
#include "robot_motion/Rotation.h"
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

using std::vector;

//HACK:グローバル宣言の変数多すぎもっと参照渡し使おう
//HACK:パラメータ関係を別のファイルにまとめる
#include<ros/ros.h>

enum class keyFlag{
  NOMAL = 0,
  ALL,
  AUTO
};

template<typename T>
struct Gyro{
  T x;
  T y;
  T z;
};

Gyro<double> gyro_robot; 

namespace safetyCheck{
  //過負荷の確認
  void torque_limit(){
    
  }
};

namespace gyroControl{
  enum class Lean : int{
    forward,
    backward,
    nomal
  };
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
    };
    switch(leanCheck(gyro_robot.z)){
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
};

//-----------------------------------------------------------------
//TODO:front_flagを機能するようにする
bool front_flag = false;
//ロボットの現在角度を取得
//rosのtfに合わせてあとで型を変える
namespace RobotState{
  void gyroCallback(const std_msgs::Float64MultiArray &msg) {
    if(msg.data.size() != 3){ROS_ERROR("gyro message is invalid");}
    gyro_robot.x = msg.data[0];
    gyro_robot.y = msg.data[1];
    gyro_robot.z = msg.data[2];
  }
  void stateManagement(){
    if(front_flag == true){
      std::swap(dynamixel_num[0], dynamixel_num[2]);
      std::swap(dynamixel_num[1], dynamixel_num[3]);
    }
  }
};
bool buttons_reverse = false;
bool flag_reset = false;
bool prev_reset = false;
std::array<double, 4> controller_key{};

keyFlag controller_state;

//TODO:コントローラ値を楽に変えられるようにする
void joyCallback(const sensor_msgs::Joy &controller) {
  buttons_reverse = controller.buttons[2];
  controller_key[0] = controller.axes[5];
  controller_key[1] = controller.axes[2];
  controller_key[2] = controller.buttons[5];
  controller_key[3] = controller.buttons[4];
  if(controller.buttons[3] == true)controller_state = keyFlag::ALL;
  if(controller.buttons[8] == true)controller_state = keyFlag::AUTO;
  if(controller.buttons[1] == true)controller_state = keyFlag::NOMAL;
  // Bキーで全てのフリッパーの角度を90°
  if ((prev_reset == false) and controller.buttons[1] == true) flag_reset = !flag_reset;
  prev_reset = controller.buttons[1];
}

ros::ServiceClient dynamixel_service;
dynamixel_workbench_msgs::DynamixelCommand srv;

bool serviceCallPos(){
  for(int i = 0; i < dynamixel_num.size(); ++i){
    srv.request.command = "_";
    srv.request.id = i + 1;
    srv.request.addr_name = "Goal_Position";
    srv.request.value = ref_DXL_raw_pos[i];
    dynamixel_service.call(srv);
  }
  //TODO:いるかあとで考える
  ros::spinOnce();
  return true;
}
bool serviceCallTheta(){
  for (int i = 0; i < dynamixel_num.size(); ++i) {
    //TODO:今のままだと計算がバグる
    srv.request.command = "_";
    srv.request.id = i + 1;
    srv.request.addr_name = "Goal_Position";
    double ref_rad = ref_DXL_rad[i] + (current_DXL_rad_raw[i] - current_DXL_rad[i]);
    srv.request.value = ref_rad * (DXLConstant::DYNAMIXEL_RESOLUTION / (M_PI * 2));
    dynamixel_service.call(srv);
  }
  ros::spinOnce();
}
//現在角度とトルクを取得
//TODO: dynamixelIDとjointstateの順番が違う
void jointStateCallback(const sensor_msgs::JointState &jointstate) {
  for(int i = 0; i < dynamixel_num.size(); ++i){
    current_DXL_rad_raw[i] = jointstate.position[dynamixel_num[i]];
    current_DXL_rad[i] = fmod(jointstate.position[dynamixel_num[i]], M_PI * 2);
    current_DXL_torque[i] = jointstate.effort[dynamixel_num[i]];
  }
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "semi_auto");
  ros::NodeHandle n;

  dynamixel_service = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
  ros::Subscriber feedback_sub = n.subscribe("/dynamixel_workbench/joint_states", 10, jointStateCallback);
  //ros::Subscriber gyro_sub = n.subscribe("gyro", 10, RobotState::gyroCallback);
  ros::Subscriber controller_sub = n.subscribe("joy", 10, joyCallback);
  ros::Rate loop_rate(400);
  semiAuto robot_model(n);
  dynamixel_workbench_msgs::DynamixelCommand srv;

  while (ros::ok()) {
    ros::spinOnce();
    switch(controller_state){
      case keyFlag::ALL:
        if ((controller_key[0] < 0) or (controller_key[1] < 0)){
          Rotation::setRotation(0, Rotation::severalType::all, Rotation::setRotationType::forward);
        }else if((controller_key[2] == true) or (controller_key[3] == true)){
          Rotation::setRotation(0, Rotation::severalType::all, Rotation::setRotationType::reverse);
        }else if(flag_reset){
          Rotation::reset();
        }else{
          Rotation::setRotation(0, Rotation::severalType::all, Rotation::setRotationType::nomal);
        }
        serviceCallPos();
        break;

      //半自動制御モード
      case keyFlag::AUTO:
        robot_model(ref_DXL_rad);
        serviceCallTheta();
        break;

      default:
        auto judge = [&](int dxl_num) -> bool{
          if(dxl_num == 0 or dxl_num == 1){
            return controller_key[dxl_num] < 0 ? true : false;
          }else if(dxl_num == 2 or dxl_num == 3){
            return controller_key[dxl_num] == true ? true : false;
          }
        };
        for(int i = 0; i < dynamixel_num.size(); ++i){
          if(judge(i) == true and buttons_reverse == true){
            Rotation::setRotation(i, Rotation::severalType::one, Rotation::setRotationType::reverse);
          }else if(judge(i) == true and buttons_reverse == false){
            Rotation::setRotation(i, Rotation::severalType::one, Rotation::setRotationType::forward);
          }else if(judge(i) == false){
            Rotation::setRotation(i, Rotation::severalType::one, Rotation::setRotationType::nomal);
          }
        }
        serviceCallPos();
        break;
    }
    loop_rate.sleep();
  }
}
