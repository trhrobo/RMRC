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
#include "robot_motion/Rotation.h"
#include "robot_motion/DXL.h"
#include "robot_motion/Constant.h"
#include "robot_motion/flipper_util.h"
//#include "dynamixel/dynamixel.h"
//#include "dynamixel.cpp"
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

//HACK:全ての変数、関数をnamespaceの中に入れる
//HACK:グローバル宣言の変数多すぎもっと参照渡し使おう
//HACK:パラメータ関係を別のファイルにまとめる
const DXL::MODE DXL_MODE = DXL::MODE::TORQUE_CONTROL;

enum class keyFlag{
  NOMAL = 0,
  ALL,
  AUTO
};

//-----------------------------------------------------------------
/*namespace safetyCheck{
  //過負荷の確認
  void torque_limit(){
  }
};

namespace gyroControl{
  enum class Lean{
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
};
*/

template<typename T>
struct Gyro{
  T x;
  T y;
  T z;
};

Gyro<double> gyro_robot; 

//ロボットの現在角度を取得
//rosのtfに合わせてあとで型を変える
/*
namespace RobotState{
  void gyroCallback(const std_msgs::Float64 &msg) {
    gyro_robot.x = msg.data.x;
    gyro_robot.y = msg.data.y;
    gyro_robot.z = msg.data.z;
  }
  void stateManagement(){
    //TODO:再考する
    //ロボットの前後を入れ替える
    /*
    if(front_flag == true){
      //配列番号を変える
      std::swap(dynamixel_num[0], dynamixel_num[2]);
      std::swap(dynamixel_num[1], dynamixel_num[3]);
    }
  }
};
*/
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
  if(controller.buttons[3] == true) controller_state = keyFlag::ALL;
  if(controller.buttons[8] == true) controller_state = keyFlag::AUTO;
  if(controller.buttons[1] == true) controller_state = keyFlag::NOMAL;
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
    if(ref_DXL_rad[i] > 360)ref_DXL_rad[i] -= 360;
    if(ref_DXL_rad[i] > 0)ref_DXL_rad[i] += 360;
    ROS_INFO("ref_DXL_rad[%d] %lf current_DXL_rad[%d] %lf", i, ref_DXL_rad[i], i, current_DXL_rad[i]);
    srv.request.command = "_";
    srv.request.id = i + 1;
    srv.request.addr_name = "Goal_Position";
    srv.request.value = DXL::dynamixelSet(ref_DXL_rad[i], current_DXL_rad[i]);
    dynamixel_service.call(srv);
  }
  ros::spinOnce();
}
//現在角度とトルクを取得
//TODO: dynamixelIDとjointstateの順番が違う
void jointStateCallback(const sensor_msgs::JointState &jointstate) {
  for(int i = 0; i < dynamixel_num.size(); ++i){
    current_DXL_raw_pos[i] = jointstate.position[dynamixel_num[i]];
    //FIXME:これでは角度を得られない(DXLの位置値を直接変換しているから一度ラジアンにしてdegToRadを使用する必要がある)
    current_DXL_rad[i] = degToRad<double>((jointstate.position[dynamixel_num[i]] + M_PI));
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
  //REVIEW:多分これだとtemplate<DXLConstant::DXL_MODE>のオブジェクトが4つ生成されない??
  
  //std::array<DXL::DXLControl<double, DXL_MODE>, 4> servo{  
  DXL::DXLControl<double, DXL_MODE> servo[4]{  
    FlipperConstant::front_right, 
    FlipperConstant::front_left, 
    FlipperConstant::rear_right, 
    FlipperConstant::rear_left
  };
  feedBackTypes feedback{false, false, true};
  semiAuto<double> robot_model(n, feedback);
  dynamixel_workbench_msgs::DynamixelCommand srv;


  while (ros::ok()) {
    for(int i = 0; i < dynamixel_num.size(); ++i){
      if(current_DXL_rad[i] <= 0)current_DXL_rad[i] = 360 + current_DXL_rad[i];
    }
    switch(controller_state){
      case keyFlag::ALL:
        if ((controller_key[0] < 0) or (controller_key[1] < 0)){
          Rotation::setRotation<Rotation::severalType::all>(0, Rotation::setRotationType::forward, servo);
        }else if((controller_key[2] == true) or (controller_key[3] == true)){
          Rotation::setRotation<Rotation::severalType::all>(0, Rotation::setRotationType::reverse, servo);
        }else if(flag_reset){
          Rotation::reset();
        }else{
          Rotation::setRotation<Rotation::severalType::all>(0, Rotation::setRotationType::nomal, servo);
        }
        break;

      //半自動制御モード
      case keyFlag::AUTO:
        robot_model(ref_DXL_rad);
        break;

      default:
        for(int i = 0; i < dynamixel_num.size(); ++i){
          if(controller_key[i] < 0){
            //TODO:reverse??
            buttons_reverse == 1 ? Rotation::setRotation<Rotation::severalType::one>(i, Rotation::setRotationType::reverse, servo) : Rotation::setRotation<Rotation::severalType::one>(i, Rotation::setRotationType::forward, servo);
          }else{
            if((i == 2 or i == 3) && controller_key[i] == true){
              buttons_reverse == 1 ? Rotation::setRotation<Rotation::severalType::one>(i, Rotation::setRotationType::reverse, servo) : Rotation::setRotation<Rotation::severalType::one>(i, Rotation::setRotationType::forward, servo);
            }
          }
        }
        break;
    }
    loop_rate.sleep();
  }
}
