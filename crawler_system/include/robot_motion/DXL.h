#ifndef DXL_H_
#define DXL_H_

#include"Rotation.h"

constexpr DXL::MODE DXL_MODE = DXL::MODE::TORQUE_CONTROL;
namespace DXL{
  enum class MODE{
    POS_CONTROL,
    TORQUE_CONTROL
  };
  template<typename T, MODE dxl_mode>
  class DXLControl(){
    public:
      DXLControl(int _ID):DXL_ID(_ID){
        if(dxl_mode == MODE::POS_CONTROL){
          //WARNING:関数ポインタの使い方が違う
          funcp = this -> PosControl;
        }else if(dxl_mode == MODE::TORQUE_CONTROL){
          //WARNING:関数ポインタの使い方が違う
          funcp = this -> TorqueControl;
        }else{
          ROS_ERROR("this dynamixel mode is not appropriate.");
        }
      }
      bool TorqueControl(T theta_d){
        //トルク制御の実装
        //重力補償項の追加
        T gravity_compensation = flipper_m * gravity * cos(degToRad<T>(theta_now));
        return DXLConstant::Kp * (degToRad<T>(theta_d) - degToRad<T>(theta_now)) - DXLConstant::Kd * (angular) + gravity_compensation;
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
      //TODO:今のままだと計算がバグる
      if(ref_DXL_rad[i] > 360)[i] -= 360;
      if(ref_DXL_rad[i] > 0)ref_DXL_rad[i] += 360;
      ROS_INFO("ref_DXL_rad[%d] %lf current_DXL_rad[%d] %lf", i, ref_DXL_rad[i], i, current_DXL_rad[i]);
      srv.request.command = "_";
      srv.request.id = i + 1;
      srv.request.addr_name = "Goal_Position";
      srv.request.value = dynamixelSet(ref_DXL_rad[i], current_DXL_rad[i]);
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
};

#endif