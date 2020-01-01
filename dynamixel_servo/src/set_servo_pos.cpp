#include "ros/time.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

using std::abs;
using std::cout;
using std::endl;

ros::Publisher arm_pub;
int plus_flag = 2;
int prev_plus_flag = 2;
double prev_value = 0;
int call = 0;
double check = 0;
vector<double> goal_angle{0, 0, 0, 0};
constexpr RANGE = 100;

std_msgs::String joint1_name;
std_msgs::Float64 joint1_pos;

enum servoStatus { FORWARD; FORWARD_ONE; REVERSE; REVERSE_ONE; DESIGNATION; }

void joyCallback(const sensor_msgs::Joy &controller) {
  if (controller.axes[5] < 0) {
    plus_flag = 1;
    check = controller.axes[5];
  }
  if (controller.buttons[5]) {
    plus_flag = 3;
  }
  check = controller.axes[5];
  call = controller.buttons[5];
}

void monitorJointState_callback(const sensor_msgs::JointState &jointstate) {
  joint1_name.data = jointstate.name[0]; // 名前読み出し
  joint1_pos.data =
      jointstate.position[0]; // ポジション読み出し    loop_rate.sleep();
  cout << "joint1_pos.data = " << joint1_pos.data << endl;
}

void forward() {}

void forwardOne() {}

void reverse() {}

void reverseOne() {}

void designation() {}

void angleCallback(const std_msgs::Float64MultiArray &msg) {}

void flagCall(const std_msgs::Bool &msg) {}

void angleCal() {
  enum servoStatus now_status;

  if (flag _designation) {
    now_status = DESIGNATION;
  } else if (flag_one) {
    flag_rotate == 1 ? now_status = FORWARD_ONE : now_status = REVERSE_ONE;
  } else {
    flag_rotate == 1 ? now_status = FORWARD : now_status = REVERSE;
  }

  switch (status) {
  case FORWARD:
    forward();
    break;
  case FORWARD_ONE:
    forwardOne();
    break;
  case REVERSE:
    reverse();
    break;
  case REVERSE_ONE:
    reverseOne();
    break;
  case DESIGNATION:
    designetaion();
    break;
  }
}

void angleConvert() {}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamixel_writer");
  ros::NodeHandle nh;
  ros::Subscriber controller_sub = nh.subscribe("joy", 100, joyCallback);
  ros::Subscriber angle_sub = nh.subscribe("servo_angle", 10, angleCallback);
  arm_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
      "/dynamixel_workbench/joint_trajectory", 100);
  ros::Subscriber sub_joints; // サブスクライバの作成
  sub_joints = nh.subscribe("/dynamixel_workbench/joint_states", 10,
                            monitorJointState_callback);
  ros::Rate loop_rate(45); // 制御周期10Hz
  trajectory_msgs::JointTrajectory jtp0;
  jtp0.header.frame_id = "base_link";
  jtp0.joint_names.resize(1);
  jtp0.points.resize(1);
  jtp0.points[0].positions.resize(1);
  jtp0.joint_names[0] = "pan";

  while (ros::ok()) {
    jtp0.header.stamp = ros::Time::now();
    ros::spinOnce(); // コールバック関数を呼ぶ
    if (joint1_pos.data + 0.5 > prev_value &&
        joint1_pos.data - 0.5 < prev_value) {
      if (plus_flag == 1) {
        jtp0.points[0].positions[0] += 2;
      } else if (plus_flag == 3) {
        jtp0.points[0].positions[0] -= 2;
      }
      if (check > 0 && call == 0) {
        jtp0.points[0].positions[0] = prev_value;
      }
    }
    cout << "prev_value = " << prev_value << endl;
    cout << "check = " << check << " "
         << "call = " << call << endl;
    jtp0.points[0].time_from_start = ros::Duration(0.02);
    jtp0.points[0].positions[0] = abs(jtp0.points[0].positions[0]);
    arm_pub.publish(jtp0);
    prev_value = jtp0.points[0].positions[0];
    prev_plus_flag = plus_flag;
    loop_rate.sleep();
    cout << "plus flag = " << plus_flag << endl;
    ROS_INFO("Joint1(%s)= %f", jtp0.joint_names[0].c_str(),
             jtp0.points[0].positions[0]);
  }

  return 0;
}
