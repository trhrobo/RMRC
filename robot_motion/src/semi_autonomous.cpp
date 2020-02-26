#include"robot_motion/semi_autonomous.h"

SemiAutonomousBase::SemiAutonomousBase(ros::NodeHandle *n){
  dynamixel_pub = n->advertise<std_msgs::Float64MultiArray>("flipper_semi_autonomous", 30);
  dynamixel_sub = n->subscribe("dynamixel_sub", 10, &SemiAutonomousBase::dynamixelCallback, this);
  gyro_sub = n->subscribe("gyro_sub", 10, &SemiAutonomousBase::gyroCallback, this);
  tof_sub = n->subscribe("tof_sub", 10, &SemiAutonomousBase::tofCallback, this);
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
  for(int i = 0; i < 4; ++i){
    tof_distance[i] = msg.data[i];
  }
}
bool SemiAutonomousBase::dynamixelLoad(){
}

SemiAutonomousFront::SemiAutonomousFront(ros::NodeHandle *n){
  poseParamFront.POSE_1 = {60, 60};
  poseParamFront.POSE_2 = {20, 20};
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

SemiAutonomousRear::SemiAutonomousRear(ros::NodeHandle *n){
  poseParamRear.POSE_1 = {60, 60};
  poseParamRear.POSE_2 = {20, 20};
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

semiAutonomous::semiAutonomous(ros::NodeHandle *n){
  front = new SemiAutonomousFront(n);
  rear = new SemiAutonomousRear(n);
}
semiAutonomous::~semiAutonomous(){
  delete front;
  delete rear;
}
void semiAutonomous::main(){
  front->mainSemiAutonomous();
  rear->mainSemiAutonomous();
}
