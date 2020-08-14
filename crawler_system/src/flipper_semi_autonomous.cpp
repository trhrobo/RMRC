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
//#include "dynamixel/dynamixel.h"
//#include "dynamixel.cpp"
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

using std::vector;

//HACK:全ての変数、関数をnamespaceの中に入れる
//HACK:グローバル宣言の変数多すぎもっと参照渡し使おう
//HACK:パラメータ関係を別のファイルにまとめる
#include<ros/ros.h>
namespace DXL{
  enum class MODE{
    POS_CONTROL,
    TORQUE_CONTROL
  };
  template<typename T, MODE dxl_mode>
  class DXLControl;
  template<typename T>
  int dynamixelSet(T goal_angle, T now_pos);
};
//----------------------------------------------------------------------------
template<typename T>
inline T degToRad(const T deg){
  return deg * (M_PI / 180.0);
}
template<typename T>
inline T radToDeg(const T rad){
  return rad * (180.0 / M_PI);
}

//----------------------------------------------------------------------------
namespace FlipperConstant{
  //HACK:一つずつサーボIDを管理するのは頭が悪い
  inline constexpr int front_right = 0;
  inline constexpr int front_left = 1;
  inline constexpr int rear_right = 2;
  inline constexpr int rear_left = 3;

  inline constexpr double flipper_m = 100;
  inline constexpr double flipper_lg = 100;
  inline constexpr double gravity = 9.81;
};
namespace DXLConstant{
  inline constexpr double ORIGIN_DEG = 90.0;
  inline constexpr double AUTO_MAX_DEG = 75.0;
  inline constexpr double AUTO_MIN_DEG = -90.0;
  inline constexpr int MAX_POSITION_VALUE = 1048575;
  inline constexpr int MIN_POSITION_VALUE = -1048575;
  inline constexpr int DYNAMIXEL_RESOLUTION = 4096;
  inline constexpr double DYNAMIXEL_RESOLUTION_ANGLE = 0.088;
  inline constexpr bool TORQUE_ENABLE = 1;
  inline constexpr bool TORQUE_DISABLE = 0;
  //TODO:それぞれのサーボゲインを設ける
  inline constexpr double Kp = 1.0;
  inline constexpr double Kd = 1.0;
};

inline std::array<double, 4>     ref_DXL_raw_pos{};
inline std::array<double, 4> current_DXL_raw_pos{};

//std::array<double, 4>     ref_DXL_rad{};
inline double ref_DXL_rad[4]{};
inline std::array<double, 4> current_DXL_rad{};

inline std::array<double, 4>     ref_DXL_torque{};
inline std::array<double, 4> current_DXL_torque{};
constexpr DXL::MODE DXL_MODE = DXL::MODE::TORQUE_CONTROL;
std::array<int, 4> dynamixel_num{0, 1, 3, 2};
//----------------------------------------------------------------------------
namespace DXL{
  /*
  enum class MODE{
    POS_CONTROL,
    TORQUE_CONTROL
  };*/
  template<typename T, MODE dxl_mode>
  class DXLControl{
    public:
      DXLControl(int _ID);
      bool TorqueControl(T theta_d);
      bool PosControl(T theta_d);
      bool PosDirect();
      bool operator()();
    private:
      const int DXL_ID;
      bool (DXLControl::*funcp)(T);
  };
  template<typename T>
  int dynamixelSet(T goal_angle, T now_pos);
};
template<typename T, DXL::MODE dxl_mode>
DXL::DXLControl<T, dxl_mode>::DXLControl(int _ID):DXL_ID(_ID){
    if(dxl_mode == DXL::MODE::POS_CONTROL){
      //WARNING:関数ポインタの使い方が違う
      funcp = &DXL::DXLControl<T, dxl_mode>::PosControl;
    }else if(dxl_mode == DXL::MODE::TORQUE_CONTROL){
      //WARNING:関数ポインタの使い方が違う
      funcp = &DXL::DXLControl<T, dxl_mode>::TorqueControl;
    }else{
      ROS_ERROR("this dynamixel mode is not appropriate.");
    }
}
template<typename T, DXL::MODE dxl_mode>
bool DXL::DXLControl<T, dxl_mode>::TorqueControl(T theta_d){
    //トルク制御の実装
    //重力補償項の追加
    //あとでangular実装
    T angular;
    T theta_now;
    T gravity_compensation = FlipperConstant::flipper_m * FlipperConstant::gravity * cos(degToRad<T>(theta_now));
    return DXLConstant::Kp * (degToRad<T>(theta_d) - degToRad<T>(theta_now)) - DXLConstant::Kd * (angular) + gravity_compensation;
}
template<typename T, DXL::MODE dxl_mode>
bool DXL::DXLControl<T, dxl_mode>::PosControl(T theta_d){
    //TODO:位置制御の追加
}
template<typename T, DXL::MODE dxl_mode>
bool DXL::DXLControl<T, dxl_mode>::PosDirect(){
    //TODO:PosDirectに追加
}
template<typename T, DXL::MODE dxl_mode>
bool DXL::DXLControl<T, dxl_mode>::operator()(){
  //FIXME:operator引数
  //theta_dはグローバル変数で宣言してるはず
  //return (*funcp)(theta_d);
  return true;
}

template<typename T>
int DXL::dynamixelSet(T goal_angle, T now_pos){
    //HACK:コードが汚い
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
    return(goal_pos / DXLConstant::DYNAMIXEL_RESOLUTION_ANGLE) + (now_pos / DXLConstant::DYNAMIXEL_RESOLUTION_ANGLE);
}
//---------------------------------------------------------------------------------------
namespace Rotation{
  enum class setRotationType{
    forward,
    reverse,
    nomal
  };
  enum class severalType{
    one,
    all
  };
  //WARNING:引数が違う気がするint idで本当にいいのか?DXLの配置位置では??
  void setRotation(const int id, const severalType type, const setRotationType direction, DXL::DXLControl<double, DXL_MODE>(&DXLservo)[dynamixel_num.size()]);

  void reset();
};

void Rotation::setRotation(const int id, const Rotation::severalType type, const Rotation::setRotationType direction, DXL::DXLControl<double, DXL_MODE>(&DXLservo)[dynamixel_num.size()]){
    if(DXL_MODE == DXL::MODE::POS_CONTROL){
        int set_id{};
        //NOTE:Type::allではid0の値を他の値にコピーする
        type == severalType::one ? set_id = id : set_id = 0;
        switch(direction){
          case setRotationType::forward:
            ref_DXL_raw_pos[set_id] = DXLConstant::MAX_POSITION_VALUE; 
            break;
          case setRotationType::reverse:
            ref_DXL_raw_pos[set_id] = DXLConstant::MIN_POSITION_VALUE;
            break;
          case setRotationType::nomal:
              ref_DXL_raw_pos[set_id] = current_DXL_raw_pos[set_id];
              break;
          default:
              ROS_ERROR("this direction of rotation is invalid");
              break;
        }
        if(type == severalType::one){
            for (int i = 1; i < dynamixel_num.size(); ++i) {
                ref_DXL_raw_pos[i] = ref_DXL_raw_pos[0];
            }
        }
        for(auto &DXL : DXLservo){
            DXL.PosDirect();
        }
    }else if(DXL_MODE == DXL::MODE::TORQUE_CONTROL){
        if(type == severalType::one){
            switch(direction){
                case setRotationType::forward:
                  break;
                case setRotationType::reverse:
                   break;
                case setRotationType::nomal:
                   break;
                default:
                   ROS_ERROR("this direction of rotation is invalid");
                   break;
            }
        }else if(type == severalType::all){
            switch(direction){
                case setRotationType::forward:
                    break;
                case setRotationType::reverse:
                    break;
                case setRotationType::nomal:
                    break;
                default:
                    ROS_ERROR("this direction of rotation is invalid");
                    break;
            }
        }
        for(auto &DXL : DXLservo){
          DXL();
        }
    }
}

void Rotation::reset() {
  for (int i = 0; i < dynamixel_num.size(); ++i) {
    //TODO:template化
    ref_DXL_rad[i] = degToRad<double>(DXLConstant::ORIGIN_DEG);
  }
}
//---------------------------------------------------------------------------
//FIXME:定数定義をスコープを付けてまとめる
namespace DXLConstant{
  constexpr int POSE_UP = 90;
  constexpr int POSE_DOWN = -90;
  constexpr int DISTANCE_THRESHOLD_FORWARD = 100;
  constexpr int DISTANCE_THRESHOLD_DOWN = 6;
}

struct DXLPose{
  vector<double> POSE_1{0, 0};
  vector<double> POSE_2{0, 0};
};

struct feedBackTypes{
  //FIXME:unionにして必ず一つの値だけを持つようにする
  //NOTE:ここにフィードバックシステムを追加する
  bool dist;
  bool pos;
  bool torque;
};

template<typename T>
class SemiAutoBase{
  protected:
    ros::Subscriber tof_sub;
    vector<T> current_DXL_pose{0, 0, 0, 0};
    vector<T> current_DXL_load{0, 0, 0, 0};
    vector<T> tof_distance{0, 0, 0, 0};
    vector<T> goal_DXL_pose{0, 0, 0, 0};
    T gyro_pose;
    T goal_pose_right;
    T goal_pose_left;
    const feedBackTypes feedback;
  public:
    SemiAutoBase(ros::NodeHandle _n, feedBackTypes _feedback):feedback(_feedback){
      tof_sub = _n.subscribe("tof_sub", 10, &SemiAutoBase::tofCallback, this);
    }
    void init(){
    }
    void tofCallback(const std_msgs::Float64MultiArray &msg){
      for(int i = 0; i < 4; ++i){
        tof_distance[i] = msg.data[i];
      }
    }
    bool DXLLoad(){
    }
    virtual void operator()(double (&set_array)[4]) = 0;
    virtual T psdCurve() = 0;
};

template<typename T>
class SemiAutoFront : public SemiAutoBase<T>{
  private:
    ros::Subscriber psd_front_sub;
    DXLPose poseParamFront;
  public:
    //FIXME:コンストラクタを修正する(多分こんなに階層的にしないでもいいはずstatic使う？？)
    SemiAutoFront(ros::NodeHandle _n, feedBackTypes _feedback) : SemiAutoBase<T>(_n, _feedback){
      poseParamFront.POSE_1 = {60, 60};
      poseParamFront.POSE_2 = {20, 20};
    }
    //TODO:psdCurveの実装をきちんとする
    T psdCurve(){
      //return -(tof_distance[0] * tof_distance[0] / 250) + 40;
    }
    void operator()(T (&set_array)[4]){
      //TODO:出力値はdeg
      /*
      bool flag_DXL_load = this -> DXLLoad();
      //FIXME:それぞれのフィードバックの紐付けが出来ていない
      if(feedback.dist){
        //NOTE:接地判定 && 閾値以内に障害物があるかどうかの判定
        if(tof_distance[1] < DISTANCE_THRESHOLD_DOWN && tof_distance[0] < DISTANCE_THRESHOLD_FORWARD){
          set_array[0] = this -> poseParamFront.POSE_1[0] - psdCurve();
          set_array[1] = this -> poseParamFront.POSE_1[1] - psdCurve();
        }else{
          set_array[0] = this -> poseParamFront.POSE_2[0];
          set_array[1] = this -> poseParamFront.POSE_2[1];
        }
      }else if(feedback.pos){
        //TODO:各フィードバック系の詳細の追加
        //NOTE:位置(角度)フィードバック
        if(feedback.dist){
          //NOTE:距離、位置(角度)フィードバック
        }
      }else if(feedback.torque){
        //NOTE:トルクフィードバック
        if(feedback.dist && feedback.pos){
          //NOTE:距離、位置(角度)、トルクフィードバック
        }else if(feedback.dist){
          //NOTE:距離、トルクフィードバック
        }else if(feedback.pos){      
          //NOTE:位置(角度)、トルクフィードバック    
        }
      }*/
    }
};

template<typename T>
class SemiAutoRear : public SemiAutoBase<T>{
  private:
    ros::Subscriber psd_rear_sub;
    DXLPose poseParamRear;
  public:
    SemiAutoRear(ros::NodeHandle _n, feedBackTypes _feedback) : SemiAutoBase<T>(_n, _feedback){
      poseParamRear.POSE_1 = {60, 60};
      poseParamRear.POSE_2 = {20, 20};
    }
    T psdCurve(){
    }
    void operator()(T (&set_array)[4]){
      //TODO:出力値はdeg
      /*
      if(feedback.dist){
          //NOTE:接地判定 && 閾値以内に障害物があるかどうかの判定
        if(tof_distance[1] > DXLConstant::DISTANCE_THRESHOLD_DOWN && tof_distance[3] > DXLConstant::DISTANCE_THRESHOLD_DOWN && tof_distance[0] > DXLConstant::DISTANCE_THRESHOLD_FORWARD){
          set_array[2] = psdCurve();
          set_array[3] = psdCurve();
        }else if(tof_distance[1] > DXLConstant::DISTANCE_THRESHOLD_DOWN|| tof_distance[0] < DXLConstant::DISTANCE_THRESHOLD_FORWARD){
          //変化しない
        }else{
          set_array[2] = this -> poseParamRear.POSE_1[0];
          set_array[3] = this -> poseParamRear.POSE_1[1];
        }    
      }else if(feedback.pos){
        //NOTE:位置(角度)フィードバック
        if(feedback.dist){
          //NOTE:距離、位置(角度)フィードバック
        }  
      }else if(feedback.torque){
        //NOTE:トルクフィードバック
        if(feedback.dist && feedback.pos){
          //NOTE:距離、位置(角度)、トルクフィードバック
        }else if(feedback.dist){
          //NOTE:距離、トルクフィードバック
        }else if(feedback.pos){      
          //NOTE:位置(角度)、トルクフィードバック    
        }
      }*/
    }
};

//FIXME:template継承を間違っているここでtemplate<typename T>するだけでいいのか？？
template<typename T>
class semiAuto{
  private:
    std::unique_ptr<SemiAutoFront<T>> front;
    std::unique_ptr<SemiAutoRear<T>> rear;
  public:
    semiAuto(ros::NodeHandle _n, feedBackTypes _feedback)
      : front(new SemiAutoFront<T>(_n, _feedback)), 
        rear(new SemiAutoRear<T>(_n, _feedback)){}
    void operator()(T (&set_array)[4]){
      (*front)(set_array);
      (*rear)(set_array);
    }
};
//-----------------------------------------------------------------------

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
          Rotation::setRotation(0, Rotation::severalType::all, Rotation::setRotationType::forward, servo);
        }else if((controller_key[2] == true) or (controller_key[3] == true)){
          Rotation::setRotation(0, Rotation::severalType::all, Rotation::setRotationType::reverse, servo);
        }else if(flag_reset){
          Rotation::reset();
        }else{
          Rotation::setRotation(0, Rotation::severalType::all, Rotation::setRotationType::nomal, servo);
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
            buttons_reverse == 1 ? Rotation::setRotation(i, Rotation::severalType::one, Rotation::setRotationType::forward, servo) : Rotation::setRotation(i, Rotation::severalType::one, Rotation::setRotationType::forward, servo);
          }else{
            if((i == 2 or i == 3) && controller_key[i] == true){
              buttons_reverse == 1 ? Rotation::setRotation(i, Rotation::severalType::one, Rotation::setRotationType::forward, servo) : Rotation::setRotation(i, Rotation::severalType::one, Rotation::setRotationType::forward, servo);
            }
          }
        }
        break;
    }
    loop_rate.sleep();
  }
}
