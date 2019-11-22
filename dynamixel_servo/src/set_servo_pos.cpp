// ~/catkin_ws/src/dynamixel_micchy/src/dynamixel_writer.cpp
#include "ros/ros.h"
#include "ros/time.h"
#include "trajectory_msgs/JointTrajectory.h"

ros::Publisher arm_pub;

int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamixel_writer"); // ノードの初期化
  ros::NodeHandle nh;                        // ノードハンドラ

  //パブリッシャの作成
  arm_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
      "/dynamixel_workbench/joint_trajectory", 1);

  ros::Rate loop_rate(10); // 制御周期10Hz

  trajectory_msgs::JointTrajectory jtp0;

  jtp0.header.frame_id = "base_link";
  jtp0.joint_names.resize(1);
  jtp0.points.resize(1);

  jtp0.points[0].positions.resize(1);

  jtp0.joint_names[0] = "pan";
  int count = 0;
  while (ros::ok()) {
    jtp0.header.stamp = ros::Time::now();
    jtp0.points[0].positions[0] =
        count; // 原点のコマンドを送り続けるので起動時にposition0に移動するだけ
    jtp0.points[0].time_from_start =
        ros::Duration(1.0); //実行時間(1秒かけて移動)
    ++count;
    arm_pub.publish(jtp0);
    ros::spinOnce(); // コールバック関数を呼ぶ
    loop_rate.sleep();
    ROS_INFO("Joint1(%s)= %f", jtp0.joint_names[0].c_str(),
             jtp0.points[0].positions[0]);
  }

  return 0;
}
