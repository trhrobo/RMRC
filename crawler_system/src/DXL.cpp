#include"robot_motion/DXL.h"
#include"robot_motion/Constant.h"
#include"robot_motion/flipper_util.h"
#include<ros/ros.h>
template<typename T, DXL::MODE dxl_mode>
DXL::DXLControl<T, dxl_mode>::DXLControl(int _ID):DXL_ID(_ID){
    if(dxl_mode == DXL::MODE::POS_CONTROL){
      //WARNING:関数ポインタの使い方が違う
      funcp = DXL::DXLControl<T, dxl_mode>::PosControl;
    }else if(dxl_mode == DXL::MODE::TORQUE_CONTROL){
      //WARNING:関数ポインタの使い方が違う
      funcp = DXL::DXLControl<T, dxl_mode>::TorqueControl;
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
    return(goal_pos / DXLConstant::DYNAMIXEL_RESOLUTION_ANGLE) + (now_pos / DXLConstant::DYNAMIXEL_RESOLUTION_ANGLE);
}
  