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
#include"imu.h"
#include"robot_motion/constant.h"
#include<array>

namespace dxl_constant{
  constexpr int pose_up = 90;
  constexpr int pose_down = -90;
  constexpr int distance_threshold_forward = 100;
  constexpr int distance_threshold_down = 6;
}

struct FlipperPose{
  std::array<double, 2> POSE_1{};
  std::array<double, 2> POSE_2{};
};

class SemiAutoBase{
  protected:
    enum class FeedBackPattern{
        pos_feedback,
        torque_feedback,
        dist_pos_feedback,
        dist_torque_feedback,
    };
    enum class FlipperMode{
      mode0, mode1, mode2, mode3, mode0_indiv, mode1_indiv, mode2_indiv, mode3_indiv
    };
    enum class ImuMode{
      mode0, mode1, mode2, mode3
    };
    FeedBackPattern _feedback_mode;
    ros::Subscriber _tof_sub;
    std::array<double, 4> _current_dxl_pose{};
    std::array<double, 4> _current_dxl_load{};
    std::array<double, 4> _tof_distance{};
    std::array<double, 4> _goal_dxl_pose{};
    double _imu_pose;
    double _goal_pose_right;
    double _goal_pose_left;
  public:
    SemiAutoBase(ros::NodeHandle _n){
      _tof_sub = _n.subscribe("tof_sub", 10, &SemiAutoBase::tofCallback, this);
      if(flag_dist == true){
        dxl_mode == dxl::Mode::pos_control ? _feedback_mode = FeedBackPattern::pos_feedback : _feedback_mode = FeedBackPattern::torque_feedback;
      }else{
        dxl_mode == dxl::Mode::pos_control ? _feedback_mode = FeedBackPattern::dist_pos_feedback : _feedback_mode = FeedBackPattern::dist_torque_feedback;
      }
    }
    void init(){
    }
    void tofCallback(const std_msgs::Float64MultiArray &msg){
      for(int i = 0; i < 4; ++i){
        _tof_distance[i] = msg.data[i];
      }
    }
    virtual void operator()(double (&set_array)[4]) = 0;
};

class SemiAutoFront : public SemiAutoBase{
  private:
    ros::Subscriber _psd_front_sub;
    FlipperPose _poseParamFront;
  public:
    SemiAutoFront(ros::NodeHandle _n) : SemiAutoBase(_n){
      //poseParamFront.POSE_1 = {, };
      //poseParamFront.POSE_2 = {, };
    }
    void operator()(double (&set_array)[4], Imu<double> &&imu){
      enum class FlipperMode{
        mode0, mode1, mode2, mode3, mode0_indiv, mode1_indiv, mode2_indiv, mode3_indiv
      };
      enum class ImuMode{
        mode0, mode1, mode2, mode3
      };
      auto touchObstacle = [=](FrontFlipper flipper) -> std::tuple<bool, bool>{
        return std::forward_as_tuple(true, true);
      }
      auto imuModeDetector = [=](){
        return ImuMode::mode0;
      }
      auto flipperModeJudge = [=](){
        bool touch_right, touch_left;
        std::tie(touch_right, touch_left) = touchObstracle();
        if((touch_right == false or touch_left == false) and imuModeDetector() == ImuMode::mode2){
          return FlipperMode::mode2;
        }
        if(touch_right == true and touch_left == true){
          return FlipperMode::mode1;
        }
        if(touch_right == true or touch_left == true){
          return FlipperMode::mode1_indiv;
        }
        return flipperMode::mode0;
      }
      switch(flipperModeJudge()){
        case FlipperMode::mode0:
          break;
        case FlipperMode::mode1:
          break;
        case FlipperMode::mode2:
          break;
        case FlipperMode::mode1_indiv:
          break;
        case FlipperMode::mode2_indiv:
          break;
      }
    }
};

class SemiAutoRear : public SemiAutoBase{
  private:
    ros::Subscriber _psd_rear_sub;
    FlipperPose _poseParamRear;
  public:
    SemiAutoRear(ros::NodeHandle _n) : SemiAutoBase(_n){
      //poseParamRear.POSE_1 = {60, 60};
      //poseParamRear.POSE_2 = {20, 20};
    }
    void operator()(double (&set_array)[4], Imu<double> &&imu){
      switch(_feedback_mode){
        case FeedBackPattern::pos_feedback:
          break;
        case FeedBackPattern::torque_feedback:
          break;
        case FeedBackPattern::dist_pos_feedback:
          break;
        case FeedBackPattern::dist_torque_feedback:
          break;
      }
    }
};

class semiAuto{
  private:
    std::unique_ptr<SemiAutoFront> _front;
    std::unique_ptr<SemiAutoRear> _rear;
    Imu<double> _imu;
  public:
    semiAuto(ros::NodeHandle _n)
      : _front(new SemiAutoFront(_n)), 
        _rear(new SemiAutoRear(_n)){}
    void operator()(double (&set_array)[4], Imu<double> &&imu){
      (*_front)(set_array, std::forward<Imu<double>>(imu));
      //完全転送されたオブジェクトにはnullptr入るはずやからこれバグるくね？？
      (*_rear)(set_array, std::forward<Imu<double>>(imu));
    }
};