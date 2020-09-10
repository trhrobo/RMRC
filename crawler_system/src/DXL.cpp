/**
 * @file DXL.cpp

 * @brief DXL.hの実装
**/
#include<ros/ros.h>
#include"robot_motion/DXL.h"
#include"robot_motion/flipper_util.h"
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
  return theta_d - fmod(now_pos, M_PI * 2);
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

//template<typename T>
//int DXL::dynamixelSet(T goal_angle, T now_pos){
//  return goal_angle - fmod(now_pos, M_PI * 2);
//}