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

class SemiAutonomousBase{
  protected:
    ros::Publisher dynamixel_pub;
    ros::Subscriber dynamixel_sub;
    ros::Subscriber gyro_sub;
    ros::Subscriber tof_sub;
    vector<double> current_dynamixel_pose{0, 0, 0, 0};
    vector<double> current_dynamixel_load{0, 0, 0, 0};
    vector<double> tof_distance{0, 0};
    vector<double> goal_dynamixel_pose{0, 0, 0, 0};
    double gyro_pose;
    double goal_pose_right;
    double goal_pose_left;
  public:
    SemiAutonomousBase();
    void init();
    void dynamixelCallback(const sensor_msgs::JointState &jointstate);
    void gyroCallback(const std_msgs::Float64 &msg);
    void tofCallback(const std_msgs::Float64MultiArray &msg);
    bool dynamixelLoad();
    virtual void mainSemiAutonomous() = 0;
    virtual double psdCurve() = 0;
};

class SemiAutonomousFront : public SemiAutonomousBase{
  private:
    ros::Subscriber psd_front_sub;
    dynamixelPose poseParamFront;
    double psd_distance_forward = 0;
    double psd_distance_down = 0;
  public:
    SemiAutonomousFront();
    void psdFrontCallback(const std_msgs::Float64MultiArray &msg);
    double psdCurve() override;
    void mainSemiAutonomous() override;
};

class SemiAutonomousRear : public SemiAutonomousBase{
  private:
    ros::Subscriber psd_rear_sub;
    dynamixelPose poseParamRear;
    double psd_distance_forward_front = 0;
    double psd_distance_down_front = 0;
    double psd_distance_forward_rear = 0;
    double psd_distance_down_rear = 0;
  public:
    SemiAutonomousRear();
    void psdRearCallback(const std_msgs::Float64MultiArray &msg);
    double psdCurve() override;
    void mainSemiAutonomous() override;
};

class semiAutonomous{
  private:
    SemiAutonomousFront *front;
    SemiAutonomousRear *rear;
  public:
    semiAutonomous();
    ~semiAutonomous();
    void main();
};
#endif
