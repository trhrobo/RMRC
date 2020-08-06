 /**
 * @file semi_autonomous.h
 * @brief 半自動制御モード
**/
#include<iostream>
#include<ros/ros.h>
#include<vector>
#include<sensor_msgs/JointState.h>
#include<std_msgs/Float64MultiArray.h>
#include<std_msgs/Float64.h>
#include<memory>

using std::vector;

constexpr int POSE_UP = 90;
constexpr int POSE_DOWN = -90;
constexpr int DISTANCE_THRESHOLD_FORWARD = 100;
constexpr int DISTANCE_THRESHOLD_DOWN = 6;

struct dynamixelPose{
  vector<double> POSE_1{0, 0};
  vector<double> POSE_2{0, 0};
};

struct feedBack{
  constexpr bool dist = false;
  constexpr bool pos = false;
  constexpr bool torque = false;
}

template<typename T>
class SemiAutoBase{
  protected:
    ros::Subscriber tof_sub;
    vector<T> current_dynamixel_pose{0, 0, 0, 0};
    vector<T> current_dynamixel_load{0, 0, 0, 0};
    vector<T> tof_distance{0, 0, 0, 0};
    vector<T> goal_dynamixel_pose{0, 0, 0, 0};
    T gyro_pose;
    T goal_pose_right;
    T goal_pose_left;
  public:
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
    virtual void operator()(double (&set_array)[4]) = 0;
    virtual T psdCurve() = 0;
};

template<typename T>
class SemiAutoFront : public SemiAutoBase{
  private:
    ros::Subscriber psd_front_sub;
    dynamixelPose poseParamFront;
  public:
    SemiAutoFront::SemiAutoFront(ros::NodeHandle _n) : SemiAutoBase(_n){
      poseParamFront.POSE_1 = {60, 60};
      poseParamFront.POSE_2 = {20, 20};
    }
    T SemiAutoFront::psdCurve(){
      return -(tof_distance[0] * tof_distance[0] / 250) + 40;
    }
    void SemiAutoFront::operator()(T (&set_array)[4]){
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
};

template<typename T>
class SemiAutoRear : public SemiAutoBase{
  private:
    ros::Subscriber psd_rear_sub;
    dynamixelPose poseParamRear;
  public:
    SemiAutoRear::SemiAutoRear(ros::NodeHandle _n) : SemiAutoBase(_n){
      poseParamRear.POSE_1 = {60, 60};
      poseParamRear.POSE_2 = {20, 20};
    }
    T SemiAutoRear::psdCurve(){
    }
    void SemiAutoRear::operator()(T (&set_array)[4]){
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
};

class semiAuto{
  private:
    std::unique_ptr<SemiAutoFront> front(_n);
    std::unique_ptr<SemiAutoRear> rear(_n);
  public:

  semiAuto(ros::NodeHandle _n):front(_n), rear(_n){
    //front = new SemiAutoFront(_n);
    //rear = new SemiAutoRear(_n);
  }

  void semiAuto::operator()(T (&set_array)[4]){
    this -> front(set_array);
    this -> rearmainSemiAuto(set_array);
  }
}
