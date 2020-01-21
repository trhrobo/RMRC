#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <vector>
using std::vector;

constexpr double original_theta = 90.0;
constexpr double autonomous_max_theta = 75.0;
vector<double> current_dynamixel_theta{0, 0, 0, 0};
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
  theta_ref[0] >= original_theta
      ? theta_ref[0] = original_theta
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
  theta_ref[2] >= original_theta
      ? theta_ref[2] = original_theta
      : theta_ref[2] < 0 ? theta_ref[2] = 0 : theta_ref[2] = theta_ref[2];
  theta_ref[3] = theta_ref[2];
}

bool judgeGrounding() { return (theta_ref > theta_rear) && (torque_rear < 0); }
}
//--------------------------------------------------------------------
namespace all {
double reset() {
  for (int i = 0; i < 4; ++i) {
    theta_ref[i] = original_theta;
  }
}
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

public:
  flipper(int user_id);
  void forward();
  void reverse();
};

flipper::flipper(int user_id) { id = user_id; }

double flipper::forward() { theta_ref[id] += 5; }

double flipper::reverse() { theta_ref[id] -= 5; }

//現在角度とトルクを取得
void jointStateCallback(const sensor_msgs::JointState &jointstate) {
  current_dynamixel_theta[0] = jointstate.position[3];
  current_dynamixel_torque[0] = jointstate.effort[3];

  current_dynamixel_theta[1] = jointstate.position[2];
  current_dynamixel_torque[1] = jointstate.effort[2];

  current_dynamixel_theta[2] = jointstate.position[1];
  current_dynamixel_torque[2] = jointstate.effort[1];

  current_dynamixel_theta[3] = jointstate.position[0];
  current_dynamixel_torque[3] = jointstate.effort[0];
}

//ロボットの現在角度を取得
void gyroCallback(const std_msgs::Float64 &msg) { gyro_robot = msg.data; }

bool buttons_reverse = false;
bool flag_semi_autonomous = false;
bool prev_semi_autonomous = false;
bool flag_all = false;
bool prev_all = false;
double axes_front_right = 0;
double axes_front_left = 0;
double buttons_rear_right = 0;
double buttons_rear_left = 0;

//コントローラ値を入力
void joyCallback(const std_msgs::Joy &controller) {
  buttons_reverse = controller.buttons[2];
  axes_front_right = controller.axes[5];
  axes_front_left = controller.axes[2];
  buttons_rear_right = controller.buttons[5];
  buttons_rear_left = controller.buttons[4];
  if ((prev_all == false) and controllet.buttons[6] == true) {
    flag_all != flag_all;
  }
  prev_all = controller.buttons[6];
  // Xboxキーが押されたらflag_semi_autonomousを切り替える
  if ((prev_semi_autonomous == false) and controllet.buttons[6] == true) {
    flag_semi_autonomous != flag_semi_autonomous;
  }
  prev_semi_autonomous = controller.buttons[6];
}

//角度PID
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
  ros::Subscriber controller_sub = n.subscribe("joy", 10, joyCallback);
  ros::Rate loop_rate(100);
  flipper position[4] = {0, 1, 2, 3};
  while (ros::ok()) {
    //半自動モードかどうか
    if (flag_semi_autonomous) {
      semi_autonomous_front::set();
      semi_autonomous_rear::set();
      //全てのフリッパーを同じように動かすか
    } else if (flag_all) {
      if ((axes_front_right < 0) or (axes_front_left < 0)) {
        all::setForward();
      }
      if ((buttons_rear_right == true) or (buttons_rear_left == true)) {
        all::setReverse();
      }
      //個別でフリッパーを動かすか
    } else {
      if (buttons_reverse) {
        if (axes_front_right < 0) {
          position[0].reverse(theta_ref[0]);
        }
        if (axes_front_left < 0) {
          position[1].reverse(theta_ref[1]);
        }
        if (buttons_rear_right) {
          position[2].reverse(theta_ref[2]);
        }
        if (buttons_rear_left) {
          position[3].reverse(theta_ref[3]);
        }
      } else {
        if (axes_front_right < 0) {
          position[0].forward(theta_ref[0]);
        }
        if (axes_front_left < 0) {
          position[1].forward(theta_ref[1]);
        }
        if (buttons_rear_right) {
          position[2].forward(theta_ref[2]);
        }
        if (buttons_rear_left) {
          position[3].forward(theta_ref[3]);
        }
      }
    }
    pidCal();
    ros::spinOnce();
    loop_rate.sleep();
  }
}
