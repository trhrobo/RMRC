/**
 * @file DXL.h

 * @brief DXLの宣言
**/
#pragma once

#include"robot_motion/Constant.h"
#include"robot_motion/flipper_util.h"
#include<ros/ros.h>
namespace DXL{
  template<typename T, DXL::MODE dxl_mode>
  class DXLControl{
    public:
      DXLControl(int);
      bool TorqueControl(T);
      bool PosControl(T);
      bool PosDirect();
      bool operator()();
    private:
      const int DXL_ID;
      bool (DXLControl::*funcp)(T);
  };
  template<typename T>
  int dynamixelSet(T, T);
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
int DXL::dynamixelSet(T goal_rad, T now_rad){
  double goal_pos = goal_rad - fmod(now_rad, M_PI * 2);
  return static_cast<int>(goal_pos);
}