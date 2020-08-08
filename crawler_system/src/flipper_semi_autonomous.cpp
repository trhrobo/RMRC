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
//FIXME:全ての変数、関数をnamespaceの中に入れる
//FIXME:グローバル宣言の変数多すぎもっと参照渡し使おう
//FIXME:パラメータ関係を別のファイルにまとめる
namespace FlipperConstant{
  //FIXME:一つずつサーボIDを管理するのは頭が悪い
  constexpr int front_right = 0;
  constexpr int front_left = 1;
  constexpr int rear_right = 2;
  constexpr int rear_left = 3;

  constexpr double flipper_m = 100;
  constexpr double flipper_lg = 100;
  constexpr double gravity = 9.81;
}
namespace DXLConstant{
  constexpr DXLControl::MODE DXL_MODE = DXLControl::MODE::TORQUE_CONTROL;
  constexpr double ORIGINAL_DEG = 90.0;
  constexpr double AUTO_MAX_DEG = 75.0;
  constexpr double AUTO_MIN_DEG = -90.0;
  constexpr int MAX_POSITION_VALUE = 1048575;
  constexpr int MIN_POSITION_VALUE = -1048575;
  constexpr int DYNAMIXEL_RESOLUTION = 4096;
  constexpr double DYNAMIXEL_RESOLUTION_ANGLE = 0.088;
  constexpr bool TORQUE_ENABLE = 1;
  constexpr bool TORQUE_DISABLE = 0;
  //FIXME:それぞれのサーボゲインを設ける
  constexpr double Kp = 1.0;
  constexpr double Kd = 1.0;
}
array<double, 4>     ref_DXL_raw_pos{};
array<double, 4> current_DXL_raw_pos{};

array<double, 4>     ref_DXL_deg{};
array<double, 4> current_DXL_deg{};

array<double, 4>     ref_DXL_torque{};
array<double, 4> current_DXL_torque{};

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

namespace Rotation{
  enum class setRotation{
    forward,
    reverse,
    nomal
  }
  enum class Type{
    one,
    all
  }
  //FIXME:引数が違う気がするint idで本当にいいのか?DXLの配置位置では??
  template<Type type>
  explicit void setRotation(const int id, const setRotation direction, DXLControl::DXLControl *DXLservo){
    if(DXLConstant::DXL_MODE == DXLControl::MODE::POS_CONTROL){
      if(type == Type::one){
        switch(direction){
          case setRotation::forward:
            ref_DXL_raw_pos[id] = DXLConstant::MAX_POSITION_VALUE; 
            break;
          case setRotation::reverse:
            ref_DXL_raw_pos[id] = DXLConstant::MIN_POSITION_VALUE;
            break;
          case setRotation::nomal:
            ref_DXL_raw_pos[id] = current_DXL_raw_pos;
            break;
          case default:
            ROS_ERROR("this direction of rotation is invalid");
            break;
        }
      }else if(type == type::all){
          switch(direction){
          case setRotation::forward:
            ref_DXL_raw_pos[0] = DXLConstant::MAX_POSITION_VALUE;
            break;
          case setRotation::reverse:
            ref_DXL_raw_pos[0] = DXLConstant::MIN_POSITION_VALUE;
            break;
          case setRotation::nomal:
            break;
          case default:
            ROS_ERROR("this direction of rotation is invalid");
            break;
        }
        for (int i = 1; i < dynamixel_num.size(); ++i) {ref_DXL_raw_pos[i] = ref_DXL_raw_pos[0];}
      }
      for(int i < 0; i < dynamixel_num.size(); ++i){
        DXLservo->PosDirect();
        ++DXLservo;
      }
    }else if(DXLConstant::DXL_MODE == DXLControl::MODE::TORQUE_CONTROL){
      if(type == Type::one){
        switch(direction){
          case setRotation::forward:
            break;
          case setRotation::reverse
            break;
          case setRotation::nomal:
            break;
          case default:
            ROS_ERROR("this direction of rotation is invalid");
            break;
        }
      }else if(type == type::all){
          switch(direction){
          case setRotation::forward:
            break;
          case setRotation::reverse:
            break;
          case setRotation::nomal:
            break;
          case default:
            ROS_ERROR("this direction of rotation is invalid");
            break;
        }
      }
      for(int i < 0; i < dynamixel_num.size(); ++i){
        DXLservo->DXLservo();
        ++DXLservo;
      }
    }
  }
  //TODO:reset()の利用
  inline void reset() {
    for (int i = 0; i < dynamixel_num.size(); ++i) {
      ref_DXL_deg[i] = DXLConstant::ORIGINAL_DEG;
    }
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

namespace DXLControl{
  enum class MODE{
    POS_CONTROL,
    TORQUE_CONTROL
  }
  template<typename T, MODE dxl_mode>
  class DXLControl(){
    public:
      explicit DXLControl(int _ID):DXL_ID(_ID){
        if(dxl_mode == MODE::POS_CONTROL){
          //FIXME:関数ポインタの使い方が違う
          funcp = this -> PosControl;
        }else if(dxl_mode == MODE::TORQUE_CONTROL){
          //FIXME:関数ポインタの使い方が違う
          funcp = this -> TorqueControl;
        }else{
          ROS_ERROR("this dynamixel mode is not appropriate.");
        }
      }
      bool TorqueControl(T theta_d){
        //トルク制御の実装
        //重力補償項の追加
        T gravity_compensation = flipper_m * gravity * cos(degToRad(theta_now));
        return radToDeg<T>(Kp * (degToRad<T>(theta_d) - degToRad<T>(theta_now)) - Kd * (angular) + gravity_compensation);
      }
      bool PosControl(T theta_d){
        //TODO:位置制御の追加
      }
      bool PosDirect(){
        //TODO:PosDirectに追加
      }
      bool operator()(T theta_d){
        return (*funcp)(theta_d);
      }
    private:
      const int DXL_ID;
      int (*funcp)(int, int);
  };
  template<typename T>
  int dynamixelSet(T goal_angle, T now_pos){
    //FIXME:コードが汚い
    //dynamixelのパルスがrosの場合だと定義が違う可能性があるので確認必要
    //現在の位置をdynamixel一回あたりのパルス数(定数で割る)
    //now_posはラジアンであるので一旦度数方に直す
    T goal_pos;
    int sum_revolutions = static_cast<int>(now_pos / 360);
    T now_angle = now_pos - (sum_revolutions * 360);
    //std::cout << "goal_angle = " << goal_angle << " now_angle = " << now_angle << std::endl;
    if(now_angle < 0)now_angle = 360 + now_angle;
    if(goal_angle - now_angle > 0 and goal_angle - now_angle < 180){
      goal_pos = goal_angle - now_angle;
    }else if(goal_angle - now_angle > 0 and goal_angle - now_angle > 180){
      goal_pos = -(now_angle + 360 - goal_angle);
    }else if(now_angle - goal_angle > 0 and now_angle - goal_angle < 180){
      goal_pos = -(now_angle - goal_angle);
    }else if(now_angle - goal_angle > 0 ){
      goal_pos = goal_angle + 360 - now_angle;
    }
    //return (goal_pos / DYNAMIXEL_RESOLUTION_ANGLE) + now_pulse + (sum_revolutions * DYNAMIXEL_RESOLUTION);
    //  std::cout << "goal_angle = " << goal_angle << " now_angle = " << now_angle << std::endl;
    return(this -> torqueFB(goal_pos / DYNAMIXEL_RESOLUTION_ANGLE) + (now_pos / DYNAMIXEL_RESOLUTION_ANGLE));
  }
  //NodeHandleとかもグローバル宣言にしたほうがいいかも
  ros::ServiceClient dynamixel_service;
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
    return 
  }
  bool serviceCallTheta(){
    for (int i = 0; i < dynamixel_num.size(); ++i) {
      if(ref_DXL_deg[i] > 360)[i] -= 360;
      if(ref_DXL_deg[i] > 0)ref_DXL_deg[i] += 360;
      ROS_INFO("ref_DXL_deg[%d] %lf current_DXL_deg[%d] %lf", i, ref_DXL_deg[i], i, current_DXL_deg[i]);
      srv.request.command = "_";
      srv.request.id = i + 1;
      srv.request.addr_name = "Goal_Position";
      srv.request.value = dynamixelSet(ref_DXL_deg[i], current_DXL_deg[i]);
      dynamixel_service.call(srv);
    }
    ros::spinOnce();
  }
  //現在角度とトルクを取得
  //FIXME: dynamixelIDとjointstateの順番が違う
  constexpr array<int, 4> dynamixel_num{0, 1, 3, 2};
  void jointStateCallback(const sensor_msgs::JointState &jointstate) {
    for(int i = 0; i < dynamixel_num.size(); ++i){
      current_DXL_raw_pos[i] = jointstate.position[dynamixel_num[i]];
      //FIXME:これでは角度を得られない(DXLの位置値を直接変換しているから一度ラジアンにしてdegToRadを使用する必要がある)
      current_DXL_deg[i] = degToRad<double>((jointstate.position[dynamixel_num[i]] + M_PI));
      current_DXL_torque[i] = jointstate.effort[dynamixel_num[i]];
    }
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
    //TODO:再考する
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

//FIXME:コントローラ値を楽に変えられるようにする
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
  ros::init(argc, argv, "semi_auto");
  ros::NodeHandle n;

  dynamixel_service = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
  ros::Subscriber feedback_sub = n.subscribe("/dynamixel_workbench/joint_states", 10, jointStateCallback);
  ros::Subscriber gyro_sub = n.subscribe("gyro", 10, gyroCallback);
  ros::Subscriber controller_sub = n.subscribe("joy", 10, joyCallback);
  ros::Rate loop_rate(400);
  //FIXME:多分これだとtemplate<DXLConstant::DXL_MODE>のオブジェクトが4つ生成されない??
  DXLControl<DXLConstant::DXL_MODE> servo[4] = {  
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
      if(current_DXL_deg[i] <= 0)current_DXL_deg[i] = 360 + current_DXL_deg[i];
    }
    switch(controller_state){
      case keyFlag::ALL:
        if ((controller_key[0] < 0) or (controller_key[1] < 0)){
          Rotation::setRotation<Rotation::Type::all>(all::forward);
        }else if((controller_key[2] == true) or (controller_key[3] == true)){
          Rotation::setRotation<Rotation::Type::all>(all::reverse);
        }else if(flag_reset){
          Rotation::reset();
        }else{
          Rotation::setRotation<Rotation::Type::all>(all::nomal);
        }
        break;

      //半自動制御モード
      case keyFlag::AUTO:
        robot_model(ref_DXL_deg);
        break;

      default:
        for(int i = 0; i < dynamixel_num.size(); ++i){
          if(controller_key[i] < 0){
            buttons_reverse == 1 ? Rotation::setRotation<Rotation::Type::one>(i, nomal::reverse) : Rotation::setRotation(i, nomal::forward);
          }else{
            if((i == 2 or i == 3) && controller_key[i] == true){
              buttons_reverse == 1 ? Rotation::setRotation<Rotation::Type::one>(i, nomal::reverse) : Rotation::setRotation(i, nomal::forward);
            }
          }
        }
        break;
    }
    loop_rate.sleep();
  }
}
