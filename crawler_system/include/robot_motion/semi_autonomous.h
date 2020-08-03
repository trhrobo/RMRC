 /**
 * @file semi_autonomous.h
 * @brief 半自動制御モード
**/
#ifndef SEMI_AUTONOMOUS_H
#define SEMI_AUTONOMOUS_H

#include<iostream>
#include<ros/ros.h>
#include<vector>
#include<sensor_msgs/JointState.h>
#include<std_msgs/Float64MultiArray.h>
#include<std_msgs/Float64.h>

using std::vector;

#define POSE_UP 90
#define POSE_DOWN -90
#define DISTANCE_THRESHOLD_FORWARD 100
#define DISTANCE_THRESHOLD_DOWN 6

typedef struct{
  vector<double> POSE_1{0, 0};
  vector<double> POSE_2{0, 0};
}dynamixelPose;

class SemiAutoBase{
  protected:
    ros::Subscriber tof_sub;
    vector<double> current_dynamixel_pose{0, 0, 0, 0};
    vector<double> current_dynamixel_load{0, 0, 0, 0};
    vector<double> tof_distance{0, 0, 0, 0};
    vector<double> goal_dynamixel_pose{0, 0, 0, 0};
    double gyro_pose;
    double goal_pose_right;
    double goal_pose_left;
  public:
    SemiAutoBase(ros::NodeHandle _n);
    void init();
    void dynamixelCallback(const sensor_msgs::JointState &jointstate);
    void gyroCallback(const std_msgs::Float64 &msg);
    void tofCallback(const std_msgs::Float64MultiArray &msg);
    bool dynamixelLoad();
    virtual void mainSemiAuto(double (&set_array)[4]) = 0;
    virtual double psdCurve() = 0;
};

class SemiAutoFront : public SemiAutoBase{
  private:
    ros::Subscriber psd_front_sub;
    dynamixelPose poseParamFront;
  public:
    SemiAutoFront(ros::NodeHandle _n);
    double psdCurve() override;
    void mainSemiAuto(double (&set_array)[4]) override;
};

class SemiAutoRear : public SemiAutoBase{
  private:
    ros::Subscriber psd_rear_sub;
    dynamixelPose poseParamRear;
  public:
    SemiAutoRear(ros::NodeHandle _n);
    double psdCurve() override;
    void mainSemiAuto(double (&set_array)[4]) override;
};

class semiAuto{
  private:
    SemiAutoFront *front;
    SemiAutoRear *rear;
  public:
    semiAuto(ros::NodeHandle _n);
    ~semiAuto();
    void main(double (&set_array)[4]);
    void poseEstimation(){}
};
#endif
