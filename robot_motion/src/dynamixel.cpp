#include"dynamixel/dynamixel.h"
#include<iostream>
#include<cmath>

template<class T>
dynamixel<T>::dynamixel(int user_id) : _id(user_id){
}

template<class T>
dynamixel<T>::~dynamixel(){
}

template<class T>
int dynamixel<T>::dynamixelSet(T goal_angle, T now_pos){
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
  return (goal_pos / DYNAMIXEL_RESOLUTION_ANGLE) + (now_pos / DYNAMIXEL_RESOLUTION_ANGLE);
}
template<class T>
T dynamixel<T>::dynamixelTimer(){
}
template<class T>
T dynamixel<T>::dynamixelReset(){
}
template<class T>
T dynamixel<T>::angleCal(T goal_value) {
  return goal_value;
}

