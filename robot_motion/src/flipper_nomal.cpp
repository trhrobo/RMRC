#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16MultiArray.h>
#include <vector>

bool buttons_reverse = false;
double axes_front_right = 0;
double axes_front_left = 0;
double axes_back_right = 0;
double axes_back_left = 0;
double current_dynamixel_pose[4]{};

enum servoStatus { front_right, front_left, back_right, back_left };

void joyCallback(const sensor_msgs::Joy &controller) {
  buttons_reverse = controller.buttons[1];
  axes_front_right = controller.axes[2];
  axes_front_left = controller.axes[2];
  axes_back_right = controller.axes[1];
  axes_back_left = controller.axes[1];
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
};

flipper::flipper(int user_id) { id = user_id; }

int flipper::forward(int value) { return value += 2; }

int flipper::reverse(int value) { return value -= 2; }

void forwardALL(int standard_value, int goal[4]) {
  goal[0] += 2;
  for (int i = 1; i < 4; ++i) {
    goal[i] = goal[0];
  }
}

void reverseALL(int standard_value, int goal[4]) {
  goal[0] -= 2;
  for (int i = 1; i < 4; ++i) {
    goal[i] = goal[0];
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "flipper");
  ros::NodeHandle n;

  ros::Publisher flipper_pub =
      n.advertise<std_msgs::Int16MultiArray>("flipper", 30);
  ros::Subscriber flipper_sub = n.subscribe("joy", 10, joyCallback);
  ros::Rate loop_rate(45);

  std_msgs::Int16MultiArray dynamixel_goal;

  int angle_goal[4]{};
  dynamixel_goal.data.resize(4);
  flipper position[4]{{front_right}, {front_left}, {back_right}, {back_left}};

  while (ros::ok()) {
    if (buttons_reverse) {
      if (axes_front_right) {
        if (current_dynamixel_pose[0] + 0.5 > angle_goal[0] &&
            current_dynamixel_pose[0] - 0.5 < angle_goal[0]) {
          angle_goal[0] = position[0].forward(angle_goal[0]);
        }
      }
      if (axes_front_left) {
        if (current_dynamixel_pose[1] + 0.5 > angle_goal[1] &&
            current_dynamixel_pose[1] - 0.5 < angle_goal[1]) {
          angle_goal[1] = position[1].forward(angle_goal[1]);
        }
      }
      if (axes_back_right) {
        if (current_dynamixel_pose[2] + 0.5 > angle_goal[2] &&
            current_dynamixel_pose[2] - 0.5 < angle_goal[2]) {
          angle_goal[2] = position[2].forward(angle_goal[2]);
        }
      }
      if (axes_back_left) {
        if (current_dynamixel_pose[3] + 0.5 > angle_goal[3] &&
            current_dynamixel_pose[3] - 0.5 < angle_goal[3]) {
          angle_goal[3] = position[3].forward(angle_goal[3]);
        }
      }
    } else {
      if (axes_front_right)
        if (current_dynamixel_pose[0] + 0.5 > angle_goal[0] &&
            current_dynamixel_pose[0] - 0.5 < angle_goal[0]) {
          angle_goal[0] = position[0].reverse(angle_goal[0]);
        }
      if (axes_front_left)
        if (current_dynamixel_pose[1] + 0.5 > angle_goal[1] &&
            current_dynamixel_pose[1] - 0.5 < angle_goal[1]) {
          angle_goal[1] = position[1].reverse(angle_goal[1]);
        }
      if (axes_back_right)
        if (current_dynamixel_pose[2] + 0.5 > angle_goal[2] &&
            current_dynamixel_pose[2] - 0.5 < angle_goal[2]) {
          angle_goal[2] = position[2].reverse(angle_goal[2]);
        }
      if (axes_back_left)
        if (current_dynamixel_pose[3] + 0.5 > angle_goal[3] &&
            current_dynamixel_pose[3] - 0.5 < angle_goal[3]) {
          angle_goal[3] = position[3].reverse(angle_goal[3]);
        }
    }
    for (int i = 0; i < 4; ++i) {
      dynamixel_goal.data[i] = angle_goal[i];
    }
    flipper_pub.publish(dynamixel_goal);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
