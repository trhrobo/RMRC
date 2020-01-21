#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <vector>

using std::vector;

constexpr double original_pose = 90.0;
vector<double> current_dynamixel_pose{0, 0, 0, 0};
vector<double> current_dynamixel_torque{0, 0, 0, 0};
vector<double> theta_ref{0, 0, 0, 0};
double gyro_robot{};
double theta_front{};
double theta_rear{};
double torque_front{};
double torque_rear{};

//------------------------------------------------------------------
namespace semi_autonomous_front {
double set() {
  if (judgeGrounding()) {
    theta_ref[0] -= 5;
  } else {
    theta_ref[0] += 5;
  }
  theta_ref[0] >= original_pose
      ? theta_ref[0] = original_pose
      : theta_ref[0] < 0 ? theta_ref[0] = 0 : theta_ref[0] = theta_ref[0];
  theta_ref[1] = theta_ref[0];
}

bool judgeGrounding() {
  return (theta_ref > theta_front) && (torque_front < 0);
}
}
//-------------------------------------------------------------------
namespace semi_autonomous_rear {
double set() {
  if (judgeGrounding()) {
    theta_ref[2] -= 5;
  } else {
    theta_ref[2] += 5;
  }
  theta_ref[2] >= original_pose
      ? theta_ref[2] = original_pose
      : theta_ref[2] < 0 ? theta_ref[2] = 0 : theta_ref[2] = theta_ref[2];
  theta_ref[3] = theta_ref[2];
}

bool judgeGrounding() { return (theta_ref > theta_rear) && (torque_rear < 0); }
}
//--------------------------------------------------------------------
namespace all {
double setForward() {
  theta_ref[0] += 5;
  for (int i = 1; i < 4; ++i) {
    theta_ref[i] = theta_ref[0];
  }
}
double setReverse() {}
theta_ref[0] -= 5;
for (int i = 1; i < 4; ++i) {
  theta_ref[i] = theta_ref[0];
}
}
//--------------------------------------------------------------------
class flipper {
private:
  int id;
  void forward();
  void reverse();

public:
  flipper(int user_id);
};

flipper::flipper(int user_id) { id = user_id; }

void flipper::forward() { theta_ref[id] += 5; }

void flipper::reverse() { theta_ref[id] -= 5; }

void jointStateCallback(const sensor_msgs::JointState &jointstate) {
  current_dynamixel_pose[0] = jointstate.position[3];
  current_dynamixel_torque[0] = jointstate.effort[3];

  current_dynamixel_pose[1] = jointstate.position[2];
  current_dynamixel_torque[1] = jointstate.effort[2];

  current_dynamixel_pose[2] = jointstate.position[1];
  current_dynamixel_torque[2] = jointstate.effort[1];

  current_dynamixel_pose[3] = jointstate.position[0];
  current_dynamixel_torque[3] = jointstate.effort[0];
}

void gyroCallback(const std_msgs::Float64 &msg) { gyro_robot = msg.data; }

//半自動モードかどうかキーを確認する
void judge_semi_autonomous() {}

void pidCal() {
  for (int i = 0; i < 4; ++i) {
    theta_ref[i] = Kp * (theta_ref[i] - theta_rear) +
                   Kd * ((theta_ref[i] - theta_rear) / frequency);
  }
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "semi_autonomous");
  ros::NodeHandle n;

  ros::Publisher semi_autonoumous_pub =
      n.advertise<std_msgs::Float64MultiArray>("flipper_semi_autonomous", 30);
  ros::Subscriber feedback_sub =
      n.subscribe("/dynamixel_workbench/joint_states", 10, jointStateCallback);
  ros::Subscriber gyro_sub = n.subscribe("gyro", 10, gyroCallback);

  ros::Rate loop_rate(100);
  while (ros::ok()) {
    //半自動モードかどうか
    if (judge_semi_autonomous()) {
      if (semi_autonomous_judge()) {
        semi_autonomous_front::set();
      } else {
        semi_autonomous_rear::set();
      }
      //全てのフリッパーを同じように動かすか
    } else if (flag_all) {
      if (judge_all) {
        all::setForward();
      } else {
        all::setReverse();
      }
      //個別でフリッパーを動かすか
    } else {
      if (judge_manual()) {
      } else {
      }
    }
    pidCal();
    ros::spinOnce();
    loop_rate.sleep();
  }
}
