#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Int16MultiArray.h>
#include <vector>

vector<double> angle_goal{0, 0, 0, 0};

enum dynamixel_name {front_right, front_left, back_right, back_left};

dynamixelCallback(const std_msgs::Int16MultiArray &msg){
  for(int i = 0; i < msg.data.size(); ++i){
    angle_goal[i] = msg.data[i];
  }
}

class dynamixel {
  private:
    int id;
  public:
    explicit dynamixel(int user_id);
    ~dynamixel();
    int angleCal(int goal_value);
}

dynamixel::dynamixel(int user_id){
  id = user_id;
}

dynamixel::~dynamixel() {}

int dynamixel::angleCal(int value) {
  return value;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamixel_pos");
  ros::NodeHandle n;

  ros::Publisher servo_pub = n.advertise<trajectory_msgs::JointTrajectory>(
      "/dynamixel_workbench/joint_trajectory", 10);

  ros::Subscriber servo_sub = n.subscribe("flipper", 45, dynamixelCallback);
  trajectory_msgs::JointTrajectory jtp0;

  jtp0.header.frame_id = "flipper";
  jtp0.joint_names.resize(4);
  jtp0.points.resize(1);

  jtp0.points[0].positions.resize(4);

  jtp0.joint_names[0] = "right_front";
  jtp0.joint_names[1] = "right_back";
  jtp0.joint_names[2] = "left_front";
  jtp0.joint_names[3] = "left_back";

  dynamixel servo[4] = {
    {front_right},
    {front_left},
    {back_right},
    {back_left}
  };

  ros::Rate loop_rate(45);

  while (ros::ok()) {
    for(int i = 0; i < 4; ++i){
      jtp0.points[0].positions[i] = servo[i].angleCal(angle_goal[i]);
    }
    servo_pos.publish(jtp0);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
