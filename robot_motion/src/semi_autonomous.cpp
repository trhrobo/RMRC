#include"robot_motion/semi_autonomous.h"

SemiAutonomousBase::SemiAutonomousBase(ros::NodeHandle _n){
  tof_sub = _n.subscribe("tof_sub", 10, &SemiAutonomousBase::tofCallback, this);
}
void SemiAutonomousBase::init(){
}
void SemiAutonomousBase::tofCallback(const std_msgs::Float64MultiArray &msg){
  for(int i = 0; i < 4; ++i){
    tof_distance[i] = msg.data[i];
  }
}
bool SemiAutonomousBase::dynamixelLoad(){
}

SemiAutonomousFront::SemiAutonomousFront(ros::NodeHandle _n) : SemiAutonomousBase(_n){
  poseParamFront.POSE_1 = {60, 60};
  poseParamFront.POSE_2 = {20, 20};
}
double SemiAutonomousFront::psdCurve(){
  return -(tof_distance[0] * tof_distance[0] / 250) + 40;
}
void SemiAutonomousFront::mainSemiAutonomous(double (&set_array)[4]){
  bool flag_dynamixel_load = this -> dynamixelLoad();
  //接地判定 && 閾値以内に障害物があるかどうかの判定
  if(tof_distance[1] < DISTANCE_THRESHOLD_DOWN && tof_distance[0] < DISTANCE_THRESHOLD_FORWARD){
    set_array[0] = this -> poseParamFront.POSE_1[0] - psdCurve();
    set_array[1] = this -> poseParamFront.POSE_1[1] - psdCurve();
  }else{
    set_array[0] = this -> poseParamFront.POSE_2[0];
    set_array[1] = this -> poseParamFront.POSE_2[1];
  }
}

SemiAutonomousRear::SemiAutonomousRear(ros::NodeHandle _n) : SemiAutonomousBase(_n){
  poseParamRear.POSE_1 = {60, 60};
  poseParamRear.POSE_2 = {20, 20};
}
double SemiAutonomousRear::psdCurve(){
}
void SemiAutonomousRear::mainSemiAutonomous(double (&set_array)[4]){
  if(tof_distance[1] > DISTANCE_THRESHOLD_DOWN && tof_distance[3] > DISTANCE_THRESHOLD_DOWN && tof_distance[0] > DISTANCE_THRESHOLD_FORWARD){
    set_array[2] = psdCurve();
    set_array[3] = psdCurve();
  }else if(tof_distance[1] > DISTANCE_THRESHOLD_DOWN|| tof_distance[0] < DISTANCE_THRESHOLD_FORWARD){
    //変化しない
  }else{
    set_array[2] = this -> poseParamRear.POSE_1[0];
    set_array[3] = this -> poseParamRear.POSE_1[1];
  }
}

semiAutonomous::semiAutonomous(ros::NodeHandle _n){
  front = new SemiAutonomousFront(_n);
  rear = new SemiAutonomousRear(_n);
}
semiAutonomous::~semiAutonomous(){
  delete front;
  delete rear;
}
void semiAutonomous::main(double (&set_array)[4]){
  front->mainSemiAutonomous(set_array);
  rear->mainSemiAutonomous(set_array);
}
