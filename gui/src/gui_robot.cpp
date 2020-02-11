#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>

vector<double> current_flipper_pos{0, 0, 0, 0};

void dynamixelCallback(const sensor_msgs::JointState &msg) {
  current_flipper_pos[0] = msg.position[3];
  current_flipper_pos[1] = msg.position[2];
  current_flipper_pos[2] = msg.position[1];
  current_flipper_pos[3] = msg.position[0];
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "gui_robot");
  ros::NodeHandle n;
  ros::Publisher joint_pub<std_msgs::JointState>("/gui_info", 10);
  ros::Subscriber dynamixel_sub =
    n.subscribe("/dynamixel_workbench/joint_states", 10, dynamixelCallback);
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    sensor_msgs::JointState js0;
    js0.header.stamp = ros::Time::now();
    js0.name.resize(4);
    js0.name[0] = "body0_joint";
    js0.name[1] = "body1_joint";
    js0.name[2] = "body2_joint";
    js0.name[3] = "body3_joint";
    js0.position.resize(4);
    // dynamixelの角度を算出する式を追加する
    js0.position[0] = -1.0 * (float)count / 40.0;
    js0.position[1] = 2.0 * (float)count / 40.0;
    js0.position[2] = 2.0 * (float)count / 40.0;
    js0.position[3] = 2.0 * (float)count / 40.0;
    joint_pub.publish(js0);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
