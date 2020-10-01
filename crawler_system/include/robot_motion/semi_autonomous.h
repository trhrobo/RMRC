 /**
 * @file semi_autonomous.h
 * @brief 半自動制御モード
**/
#pragma once

#include<iostream>
#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<std_msgs/Float64MultiArray.h>
#include<std_msgs/Float64.h>
#include<memory>
#include"flipper_util.h"
#include"robot_motion/constant.h"
#include<array>

namespace dxl_constant{
  constexpr int pose_up = 90;
  constexpr int pose_down = -90;
  constexpr int distance_threshold_forward = 100;
  constexpr int distance_threshold_down = 6;
}

struct DXLPose{
  std::array<double, 2> POSE_1{};
  std::array<double, 2> POSE_2{};
};

class SemiAutoBase{
  protected:
    enum class FB_Pattern{
        posFB,
        torqueFB,
        dist_posFB,
        dist_torqueFB,
    };
    FB_Pattern FB_MODE;
    ros::Subscriber tof_sub;
    std::array<double, 4> current_dxl_pose{};
    std::array<double, 4> current_dxl_load{};
    std::array<double, 4> tof_distance{};
    std::array<double, 4> goal_dxl_pose{};
    double gyro_pose;
    double goal_pose_right;
    double goal_pose_left;
  public:
    SemiAutoBase(ros::NodeHandle _n){
      tof_sub = _n.subscribe("tof_sub", 10, &SemiAutoBase::tofCallback, this);
      if(flag_dist == true){
        dxl_mode == dxl::Mode::pos_control ? FB_MODE = FB_Pattern::posFB : FB_MODE = FB_Pattern::torqueFB;
      }else{
        dxl_mode == dxl::Mode::pos_control ? FB_MODE = FB_Pattern::dist_posFB : FB_MODE = FB_Pattern::dist_torqueFB;
      }
    }
    void init(){
    }
    void tofCallback(const std_msgs::Float64MultiArray &msg){
      for(int i = 0; i < 4; ++i){
        tof_distance[i] = msg.data[i];
      }
    }
    bool DXLLoad(){
    }
    virtual void operator()(double (&set_array)[4]) = 0;
};

class SemiAutoFront : public SemiAutoBase{
  private:
    ros::Subscriber psd_front_sub;
    DXLPose poseParamFront;
  public:
    SemiAutoFront(ros::NodeHandle _n) : SemiAutoBase(_n){
      //poseParamFront.POSE_1 = {, };
      //poseParamFront.POSE_2 = {, };
    }
    void operator()(double (&set_array)[4]){
      switch(FB_MODE){
        case FB_Pattern::posFB:
          break;
        case FB_Pattern::torqueFB:
          break;
        case FB_Pattern::dist_posFB:
          break;
        case FB_Pattern::dist_torqueFB:
          break;
      }
    }
};

class SemiAutoRear : public SemiAutoBase{
  private:
    ros::Subscriber psd_rear_sub;
    DXLPose poseParamRear;
  public:
    SemiAutoRear(ros::NodeHandle _n) : SemiAutoBase(_n){
      //poseParamRear.POSE_1 = {60, 60};
      //poseParamRear.POSE_2 = {20, 20};
    }
    void operator()(double (&set_array)[4]){
      switch(FB_MODE){
        case FB_Pattern::posFB:
          break;
        case FB_Pattern::torqueFB:
          break;
        case FB_Pattern::dist_posFB:
          break;
        case FB_Pattern::dist_torqueFB:
          break;
      }
    }
};

class semiAuto{
  private:
    std::unique_ptr<SemiAutoFront> front;
    std::unique_ptr<SemiAutoRear> rear;
  public:
    semiAuto(ros::NodeHandle _n)
      : front(new SemiAutoFront(_n)), 
        rear(new SemiAutoRear(_n)){}
    void operator()(double (&set_array)[4]){
      (*front)(set_array);
      (*rear)(set_array);
    }
};