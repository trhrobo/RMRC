#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16MultiArray.h>
#include <vector>
#include <comprehensive/Button.h>

bool buttons_reverse = false;
bool flag_change_front = false;
double axes_front_right = 0;
double axes_front_left = 0;
double axes_back_right = 0;
double axes_back_left = 0;
double current_dynamixel_pose[4]{};

enum servoStatus{front_right, front_left, back_right, back_left}

void joyCallback(const sensor_msgs::Joy &controller){
  buttons_reverse = controller.buttons[x];
  axes_front_right = controller.axes[r2];
  axes_front_left = controller.axes[l2];
  axes_back_right = controller.axes[r1];
  axes_back_left = controller.axes[l1];
}

void changeCallback(const comprehensive::Button &msg){
  flag_change_front = msg.change;
}

void jointStateCallback(const sensor_msgs::JointState &jointstate) {
  for (int i = 0; i < 4; ++i) {
    current_dynamixel_pose[i] = jointstate.position[i];
  }
}

class flipper {
private:
  int id;

public:
  flipper(int user_id);
  int forward(int value);
  int reverse(int value);
}

flipper::flipper(int user_id) {
  id = user_id;
}

int flipper::front(int value) { return value += 2; }

int flipper::reverse(int value) { return value -= 2; }

void forwardALL(int standard_value, goal[4]) {
  goal[0] += 2;
  for (int i = 1; i < 4; ++i) {
    goal[i] = goal[0];
  }
}

void reverseALL(int standard_value, goal[4]) {
  goal[0] -= 2;
  for (int i = 1; i < 4; ++i) {
    goal[i] = goal[0];
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "flipper");
  ros::NodeHandle n;

  ros::Publisher flipper_pub = n.advertise<std_msgs::Int16MultiArray>("flipper");
  ros::Subscriber flipper_sub = n.subscribe("joy", 10, joyCallback);
  ros::Subscriber change_sub = n.subscribe("xbox", 10, changeCallback);
  ros::Rate loop_rate(45);

  std_msgs::Int16MultiArray dynamixel_goal;
  trajectory_msgs::JointTrajectory jtp0;
  jtp0.header.frame_id = "base_link";
  jtp0.joint_names.resize(4);
  jtp0.points.resize(4);
  jtp0.points[0].positions.resize(4);
  jtp0.joint_names[0] = "front_right";
  jtp0.joint_names[1] = "front_left";
  jtp0.joint_names[2] = "back_right";
  jtp0.joint_names[3] = "back_left";

  int angle_goal[4]{};
  dynamixel_goal.data.resize(4);
  flipper position[4]{{front_right}, {front_left}, {back_right}, {back_left}};

  while (ros::ok()) {
    if (buttons_reverse) {
      if (axes_front_right) {
        if (current_dynamixel_pose[0] + 0.5 > angle_goal[0] && current_dynamixel_pose[0] - 0.5 < angle_goal[0]) {
          angle_goal[0] = position[0].forward(angle_goal[0]);
        }
      }
      if (axes_front_left) {
        if (current_dynamixel_pose[1] + 0.5 > angle_goal[1] && current_dynamixel_pose[1] - 0.5 < angle_goal[1]) {
          angle_goal[1] = position[1].forward(angle_goal[1]);
        }
      }
      if (axes_back_right) {
        if (current_dynamixel_pose[2] + 0.5 > angle_goal[2] && current_dynamixel_pose[2] - 0.5 < angle_goal[2]) {
          angle_goal[2] = position[2].forward(angle_goal[2]);
        }
      }
      if (axes_back_left) {
        if (current_dynamixel_pose[3] + 0.5 > angle_goal[3] && current_dynamixel_pose[3] - 0.5 < angle_goal[3]) {
          angle_goal[3] = position[3].forward(angle_goal[3]);
        }
      }
    } else {
      if (axes_front_right)
        if (current_dynamixel_pose[0] + 0.5 > angle_goal[0] && current_dynamixel_pose[0] - 0.5 < angle_goal[0]) {
          angle_goal[0] = position[0].reverse(angle_goal[0]);
        }
      if (axes_front_left)
        if (current_dynamixel_pose[1] + 0.5 > angle_goal[1] && current_dynamixel_pose[1] - 0.5 < angle_goal[1]) {
          angle_goal[1] = position[1].reverse(angle_goal[1]);
        }
      if (axes_back_right)
        if (current_dynamixel_pose[2] + 0.5 > angle_goal[2] && current_dynamixel_pose[2] - 0.5 < angle_goal[2]) {
          angle_goal[2] = position[2].reverse(angle_goal[2]);
        }
      if (axes_back_left)
        if (current_dynamixel_pose[3] + 0.5 > angle_goal[3] && current_dynamixel_pose[3] - 0.5 < angle_goal[3]) {
          angle_goal[3] = position[3].reverse(angle_goal[3]);
        }
    }
    for (int i = 0; i < 4; ++i) {
      dynamixel_goal[i].data = angle_goal[i];
    }
    flipper_pub.publish(jtp0);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
