#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

enum dynamixel_name { right_front, right_back, left_front, left_back };

class dynamixel {
private:
  int id;

public:
  explicit dynamixel(int user_id);
  ~dynamixel();
  int servoSet(int goal_value);
}

dynamixel::dynamixel(int user_id, string position) {
  id = user_id;
  switch (position) {
  case right_front:
    break;
  case right_back:
    break;
  case left_front:
    break;
  case left_back:
    break;
  }
}

dynamixel::~dynamixel() {}

dynamixel::servoSet(int value) {}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamixel_pos");
  ros::NodeHandle n;

  ros::Publisher servo_pub = n.advertise<trajectory_msgs::JointTrajectory>(
      "/dynamixel_workbench/joint_trajectory", 10);
  trajectory_msgs::JointTrajectory jtp0;

  jtp0.header.frame_id = "flipper";
  jtp0.joint_names.resize(4);
  jtp0.points.resize(1);

  jtp0.points[0].positions.resize(4);

  jtp0.joint_names[0] = "right_front";
  jtp0.joint_names[0] = "right_back";
  jtp0.joint_names[0] = "left_front";
  jtp0.joint_names[0] = "left_back";

  ros::Rate loop_rate(45);

  while (ros::ok()) {
    for (const auto &value : jtp0.points[0].positions) {
      value = goal;
    }
    servo_pos.publish(jtp0);
    ros::spinOnce;
    loop_rate.sleep();
  }
}
