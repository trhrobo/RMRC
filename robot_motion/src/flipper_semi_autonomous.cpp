#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include "robot_motion/semi_autonomous.h"
using std::vector;

constexpr double original_theta = 90.0;
constexpr double autonomous_max_theta = 75.0;
constexpr double autonomous_min_theta = -90.0;
constexpr double Kp = 1.0;
constexpr double Kd = 1.0;
constexpr double frequency = 45;
vector<double> current_dynamixel_theta{0, 0, 0, 0};
vector<double> current_dynamixel_torque{0, 0, 0, 0};
double theta_ref[4]{0, 0, 0, 0};
double gyro_robot{};
double theta_front{};
double theta_rear{};
double torque_front{};
double torque_rear{};

#define DEBUG 1

//------------------------------------------------------------------
namespace all {
  bool check() {
    for (int i = 0; i < 4; ++i) {
      if ((current_dynamixel_theta[i] + 0.5 > theta_ref[i]) and (current_dynamixel_theta[i] - 0.5 < theta_ref[i])) {
      } else {
        return false;
      }
    }
    return true;
  }

  inline double reset() {
    for (int i = 0; i < 4; ++i) {
      theta_ref[i] = original_theta;
    }
  }
  inline double setForward() {
    if (all::check()) {
      theta_ref[0] += 1.3;
    }
    for (int i = 1; i < 4; ++i) {
      theta_ref[i] = theta_ref[0];
    }
  }
  inline double setReverse() {
    if (all::check()) {
      theta_ref[0] -= 1.3;
    }
    for (int i = 1; i < 4; ++i) {
      theta_ref[i] = theta_ref[0];
    }
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

void flipper::forward() {
  if ((current_dynamixel_theta[id] + 0.5 > theta_ref[id]) and (current_dynamixel_theta[id] - 0.5 < theta_ref[id])) {
#ifdef DEBUG
    ROS_INFO("OK_FORWARD %d", id);
    ROS_INFO("theta_ref[%d] %lf current_dynamixel_theta[%d] %lf", id, theta_ref[id], id, current_dynamixel_theta[id]);
#endif
    theta_ref[id] += 1.3;
#ifdef DEBUG
    ROS_INFO("theta_ref_result[%d] %lf", id, theta_ref[id]);
#endif
  }
}

void flipper::reverse() {
  if ((current_dynamixel_theta[id] + 0.5 > theta_ref[id]) and (current_dynamixel_theta[id] - 0.5 < theta_ref[id])) {
#ifdef DEBUG
    ROS_INFO("OK_REVERSE %d", id);
    ROS_INFO("theta_ref[%d] %lf current_dynamixel_theta[%d] %lf", id, theta_ref[id], id, current_dynamixel_theta[id]);
#endif
    theta_ref[id] -= 1.3;
  }
}

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
bool flag_reset = false;
bool prev_reset = false;
double axes_front_right = 0;
double axes_front_left = 0;
double buttons_rear_right = 0;
double buttons_rear_left = 0;

//コントローラ値を入力
void joyCallback(const sensor_msgs::Joy &controller) {
  buttons_reverse = controller.buttons[2];
  axes_front_right = controller.axes[5];
  axes_front_left = controller.axes[2];
  buttons_rear_right = controller.buttons[5];
  buttons_rear_left = controller.buttons[4];
  if ((prev_all == false) and controller.buttons[3] == true) {
    flag_all = !flag_all;
  }
  prev_all = controller.buttons[3];
  // Xboxキーが押されたらflag_semi_autonomousを切り替える
  if ((prev_semi_autonomous == false) and controller.buttons[8] == true) {
    flag_semi_autonomous = !flag_semi_autonomous;
  }
  prev_semi_autonomous = controller.buttons[8];
  // Bキーで全てのフリッパーの角度を90°
  if ((prev_reset == false) and controller.buttons[1] == true) {
    flag_reset = !flag_reset;
  }
  prev_reset = controller.buttons[1];
#ifdef DEBUG
  ROS_INFO("flag_all %d flag_semi_autonomous %d", flag_all, flag_semi_autonomous);
#endif
}

//角度PID
inline void pidCal() {
  for (int i = 0; i < 4; ++i) {
    theta_ref[i] = Kp * (theta_ref[i] - theta_rear) +
      Kd * ((theta_ref[i] - theta_rear) / frequency);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "semi_autonomous");
  ros::NodeHandle n;

  ros::Publisher semi_autonomous_pub = n.advertise<std_msgs::Float64MultiArray>("flipper_semi_autonomous", 30);
  ros::Subscriber feedback_sub = n.subscribe("/dynamixel_workbench/joint_states", 10, jointStateCallback);
  ros::Subscriber gyro_sub = n.subscribe("gyro", 10, gyroCallback);
  ros::Subscriber controller_sub = n.subscribe("joy", 10, joyCallback);
  ros::Rate loop_rate(45);
  flipper position[4] = {0, 1, 2, 3};
  semiAutonomous robot_model(n);
  std_msgs::Float64MultiArray send;
  send.data.resize(4);

  while (ros::ok()) {
    //半自動モードかどうか
    if (flag_semi_autonomous) {
      robot_model.main(theta_ref);
      //全てのフリッパーを同じように動かすか
    } else if (flag_all) {
      if ((axes_front_right < 0) or (axes_front_left < 0)) {
        all::setForward();
      }
      if ((buttons_rear_right == true) or (buttons_rear_left == true)) {
        all::setReverse();
      }
      if (flag_reset) {
        all::reset();
      }
      //個別でフリッパーを動かすか
    } else {
      if (buttons_reverse) {
        if (axes_front_right < 0) {
          position[0].reverse();
        }
        if (axes_front_left < 0) {
          position[1].reverse();
        }
        if (buttons_rear_right) {
          position[2].reverse();
        }
        if (buttons_rear_left) {
          position[3].reverse();
        }
      } else {
        if (axes_front_right < 0) {
          position[0].forward();
        }
        if (axes_front_left < 0) {
          position[1].forward();
        }
        if (buttons_rear_right) {
          position[2].forward();
        }
        if (buttons_rear_left) {
          position[3].forward();
        }
      }
    }
    // pidCal();
    for (int i = 0; i < 4; ++i) {
      send.data[i] = theta_ref[i];
    }
    theta_front = theta_ref[0];
    theta_rear = theta_ref[2];
    semi_autonomous_pub.publish(send);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
