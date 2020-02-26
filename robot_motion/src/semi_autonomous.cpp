#include"robot_motion/semi_autonomous.h"

SemiAutonomousBase::SemiAutonomousBase(){
  ros::NodeHandle n;
  dynamixel_pub = n.advertise<std_msgs::Float64MultiArray>("flipper_semi_autonomous", 30);
  dynamixel_sub = n.subscribe("dynamixel_sub", 10, &SemiAutonomousBase::dynamixelCallback, this);
  gyro_sub = n.subscribe("gyro_sub", 10, &SemiAutonomousBase::gyroCallback, this);
  tof_sub = n.subscribe("tof_sub", 10, &SemiAutonomousBase::tofCallback, this);
}
void SemiAutonomousBase::init(){
}
void SemiAutonomousBase::dynamixelCallback(const sensor_msgs::JointState &jointstate){
  for(int i = 0; i < 4; ++i){
    current_dynamixel_pose[i] = jointstate.position[3 - i];
    current_dynamixel_load[i] = jointstate.position[3 - i];
  }
}
void SemiAutonomousBase::gyroCallback(const std_msgs::Float64 &msg){
  gyro_pose = msg.data;
}
void SemiAutonomousBase::tofCallback(const std_msgs::Float64MultiArray &msg){
  for(int i = 0; i < 2; ++i){
    tof_distance[i] = msg.data[i];
  }
}
bool SemiAutonomousBase::dynamixelLoad(){
}

SemiAutonomousFront::SemiAutonomousFront(){
  psd_front_sub = n.subscribe("psd_info", 10, psdFrontCallback);
  poseParamFront.POSE_1 = {60, 60};
  poseParamFront.POSE_2 = {20, 20};
}
void SemiAutonomousFront::psdFrontCallback(const std_msgs::Float64MultiArray &msg){
  psd_distance_forward = msg.data[0];
  psd_distance_down = msg.data[1];
}
double SemiAutonomousFront::psdCurve(){
  return -(psd_distance_forward * psd_distance_forward / 250) + 40;
}
void SemiAutonomousFront::mainSemiAutonomous(){
  bool flag_dynamixel_load = this -> dynamixelLoad();
  //接地判定 && 閾値以内に障害物があるかどうかの判定
  if(psd_distance_down < DISTANCE_THRESHOLD_DOWN && psd_distance_forward < DISTANCE_THRESHOLD_FORWARD){
    goal_pose_right = this -> poseParamFront.POSE_1[0] - psdCurve();
    goal_pose_left = this -> poseParamFront.POSE_1[1] - psdCurve();
  }else{
    goal_pose_right = this -> poseParamFront.POSE_2[0];
    goal_pose_left = this -> poseParamFront.POSE_2[1];
  }
}

SemiAutonomousRear::SemiAutonomousRear(){
  psd_rear_sub = n.subscribe("psd_info", 10, psdRearCallback);
  poseParamRear.POSE_1 = {60, 60};
  poseParamRear.POSE_2 = {20, 20};
}
void SemiAutonomousRear::psdRearCallback(const std_msgs::Float64MultiArray &msg){
  psd_distance_forward_front = msg.data[0];
  psd_distance_down_front = msg.data[1];
  psd_distance_forward_rear = msg.data[2];
  psd_distance_down_rear = msg.data[3];
}
double SemiAutonomousRear::psdCurve(){
}
void SemiAutonomousRear::mainSemiAutonomous(){
  if(psd_distance_down_front > DISTANCE_THRESHOLD_DOWN && psd_distance_down_rear > DISTANCE_THRESHOLD_DOWN && psd_distance_forward_front > DISTANCE_THRESHOLD_FORWARD){
    goal_pose_right = psdCurve();
    goal_pose_left = psdCurve();
  }else if(psd_distance_down_front > DISTANCE_THRESHOLD_DOWN|| psd_distance_forward_front < DISTANCE_THRESHOLD_FORWARD){
    //変化しない
  }else{
    goal_pose_right = this -> poseParamRear.POSE_1[0];
    goal_pose_left = this -> poseParamRear.POSE_1[1];
  }
}

semiAutonomous::semiAutonomous(){
  front = new SemiAutonomousFront;
  rear = new SemiAutonomousRear;
}
semiAutonomous::~semiAutonomous(){
  delete front;
  delete rear;
}
void semiAutonomous::main(){
  front->mainSemiAutonomous();
  rear->mainSemiAutonomous();
}
