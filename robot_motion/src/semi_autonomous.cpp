#include"robot_motion/semi_autonomous.h"

SemiAutonomousBase::SemiAutonomousBase(){
  tof_sub = n->subscribe("tof_sub", 10, &SemiAutonomousBase::tofCallback, this);
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

SemiAutonomousFront::SemiAutonomousFront(double *set_array, int size) : SemiAutonomousBase(){
  poseParamFront.POSE_1 = {60, 60};
  poseParamFront.POSE_2 = {20, 20};
  *goal_pose_right = set_array[0];
  *goal_pose_left = set_array[1];
}
double SemiAutonomousFront::psdCurve(){
  return -(tof_distance[0] * tof_distance[0] / 250) + 40;
}
void SemiAutonomousFront::mainSemiAutonomous(){
  bool flag_dynamixel_load = this -> dynamixelLoad();
  //接地判定 && 閾値以内に障害物があるかどうかの判定
  if(tof_distance[1] < DISTANCE_THRESHOLD_DOWN && tof_distance[0] < DISTANCE_THRESHOLD_FORWARD){
    goal_pose_right = this -> poseParamFront.POSE_1[0] - psdCurve();
    goal_pose_left = this -> poseParamFront.POSE_1[1] - psdCurve();
  }else{
    goal_pose_right = this -> poseParamFront.POSE_2[0];
    goal_pose_left = this -> poseParamFront.POSE_2[1];
  }
}

SemiAutonomousRear::SemiAutonomousRear(double *set_array, int size) : SemiAutonomousBase(){
  poseParamRear.POSE_1 = {60, 60};
  poseParamRear.POSE_2 = {20, 20};
  *goal_pose_right = set_array[2];
  *goal_pose_left = set_array[3];
}
double SemiAutonomousRear::psdCurve(){
}
void SemiAutonomousRear::mainSemiAutonomous(){
  if(tof_distance[1] > DISTANCE_THRESHOLD_DOWN && tof_distance[3] > DISTANCE_THRESHOLD_DOWN && tof_distance[0] > DISTANCE_THRESHOLD_FORWARD){
    goal_pose_right = psdCurve();
    goal_pose_left = psdCurve();
  }else if(tof_distance[1] > DISTANCE_THRESHOLD_DOWN|| tof_distance[0] < DISTANCE_THRESHOLD_FORWARD){
    //変化しない
  }else{
    goal_pose_right = this -> poseParamRear.POSE_1[0];
    goal_pose_left = this -> poseParamRear.POSE_1[1];
  }
}

semiAutonomous::semiAutonomous(double *set_array, int size){
  front = new SemiAutonomousFront(set_array, size);
  rear = new SemiAutonomousRear(set_array, size);
}
semiAutonomous::~semiAutonomous(){
  delete front;
  delete rear;
}
void semiAutonomous::main(){
  front->mainSemiAutonomous();
  rear->mainSemiAutonomous();
}
