/**
 * @file semi_autonomous.cpp

 * @brief semi_autonomous.hの実装
**/

SemiAutoBase::SemiAutoBase(ros::NodeHandle _n){
  tof_sub = _n.subscribe("tof_sub", 10, &SemiAutoBase::tofCallback, this);
}
void SemiAutoBase::init(){
}
void SemiAutoBase::tofCallback(const std_msgs::Float64MultiArray &msg){
  for(int i = 0; i < 4; ++i){
    tof_distance[i] = msg.data[i];
  }
}
bool SemiAutoBase::dynamixelLoad(){
}

SemiAutoFront::SemiAutoFront(ros::NodeHandle _n) : SemiAutoBase(_n){
  poseParamFront.POSE_1 = {60, 60};
  poseParamFront.POSE_2 = {20, 20};
}
double SemiAutoFront::psdCurve(){
  return -(tof_distance[0] * tof_distance[0] / 250) + 40;
}
void SemiAutoFront::operator()(double (&set_array)[4]){
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

SemiAutoRear::SemiAutoRear(ros::NodeHandle _n) : SemiAutoBase(_n){
  poseParamRear.POSE_1 = {60, 60};
  poseParamRear.POSE_2 = {20, 20};
}
double SemiAutoRear::psdCurve(){
}
void SemiAutoRear::operator()(double (&set_array)[4]){
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

semiAuto::semiAuto(ros::NodeHandle _n){
  std::unique_ptr<SemiAutoFront> front(_n);
  std::unique_ptr<SemiAutoRear> rear(_n);
  //front = new SemiAutoFront(_n);
  //rear = new SemiAutoRear(_n);
}

void semiAuto::operator()(double (&set_array)[4]){
  this -> front(set_array);
  this -> rearmainSemiAuto(set_array);
}
