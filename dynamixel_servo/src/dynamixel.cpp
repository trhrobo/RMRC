#include"dynamixel/dynamixel.h"
#include<iostream>
#include<cmath>

dynamixel::dynamixel(int user_id) : _id(user_id){
}

dynamixel::~dynamixel(){
}

int dynamixel::dynamixelSet(double goal_angle, double now_pos){
  //dynamixelのパルスがrosの場合だと定義が違う可能性があるので確認必要
  //現在の位置をdynamixel一回あたりのパルス数(定数で割る)
  //now_posはラジアンであるので一旦度数方に直す
  double now_pos_deg = (now_pos + M_PI) * 180 / M_PI;
  double goal_pos;
  int sum_revolutions = static_cast<int>(now_pos_deg / 360);
  //std::cout << "now_pos_deg = " << now_pos_deg << " now_pos = " << now_pos << std::endl;
  double now_angle = now_pos_deg - (sum_revolutions * 360);
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
  std::cout << "goal_angle = " << goal_angle << " now_angle = " << now_angle << std::endl;
  return (goal_pos / DYNAMIXEL_RESOLUTION_ANGLE) + (now_pos_deg / DYNAMIXEL_RESOLUTION_ANGLE);
}
double dynamixel::dynamixelTimer(){
}
double dynamixel::dynamixelReset(){
}
double dynamixel::angleCal(double goal_value) {
  return goal_value;
}
