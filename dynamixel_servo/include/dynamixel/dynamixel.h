#ifndef dynamixel
#define dynamixel

#include<array>

#define DYNAMIXEL_RESOLUTION 4096
#define DYNAMIXEL_RESOLUTION_ANGLE 0.088

typedef struct{
  std::array<int, 3> param;
}dynamixelParam;

class dynamixel {
private:
  int _id;
public:
  dynamixel(int user_id):_id(user_id){
  }
  ~dynamixel();
  //goal_posの範囲は0-360まで
  double dynamixelSet(double goal_angle, double now_pos){
    //dynamixelのパルスがrosの場合だと定義が違う可能性があるので確認必要
    //現在の位置をdynamixel一回あたりのパルス数(定数で割る)
    double goal_pos;
    int sum_revolutions = static_cast<int>(now_pos / DYNAMIXEL_RESOLUTION);
    double now_pulse -= sum_revolutions * DYNAMIXEL_RESOLUTION;
    double now_angle = now_pulse * DYNAMIXEL_RESOLUTION_ANGLE;
    if(goal_angle > now_angle){
      //ラムダ式で書き換える
      goal_angle - now_angle > now_angle + 360 - goal_angle ? goal_pos = now_angle + 360 - goal_pos
                                                            : goal_pos = -(goal_angle - now_angle);
    }else{
      now_angle - goal_angle > goal_angle + 360 - now_angle ? goal_pos = goal_angle + 360 - now_angle
                                                            : goal_pos = -(now_angle - goal_angle);
    }
    return (goal_pos / DYNAMIXEL_RESOLUTION_ANGLE) + now_pulse + (sum_revolutions * DYNAMIXEL_RESOLUTION);
  }
  double dynamixelTimer(){
  }
  double dynamixelReset(){
  }
  double angleCal(double goal_value) { return goal_value; }
};

#endif
