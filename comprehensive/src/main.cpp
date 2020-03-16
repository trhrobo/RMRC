#include <cmath>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <sensor_msgs/Joy.h>
#include <utility>

using std::vector;
using std::abs;
using std::swap;

double speed = 0;
double angle = 0;
vector<double> wheel_speed{0, 0};
float gyro_data{};

void velCallback(const geometry_msgs::Twist &vel) {
  speed = hypot(vel.linear.y, vel.linear.x);
  angle = atan2(vel.linear.x, vel.linear.y);
}
void gyroCallback(const std_msgs::Float32MultiArray &msg){
  gyro_data = msg.data;
}
bool flag_change = false;
bool flag_change_prev = false;
void joyCallback(const sensor_msgs::Joy &msg){
    if(flag_change_prev == false && msg.buttons[6] == true){
        flag_change = !flag_change;
    }
    flag_change_prev = msg.buttons[6];
}
double turn_right, turn_left;

float angle_data(){
  static float prev_value{};
  static float cal_diff{};
  static array<float, 10> get_value{};
  static float goal_prev{};
  static float goal_now{};
  static float goal_diff{};
  static float goal_diff_sum{};
  static float deviation{};
  if(i == 10){
    cal_diff = get_value[9] - prev_value;
    prev_vlaue = get_value[9];
    get_value.fill(0);
    deviation = cal_diff - goal_diff_sum;
    goal_diff_sum = 0;
  }
  //goal_nowを用意する
  goal_diff = goal_now - goal_prev;
  goal_diff_sum += goal_diff;
  goal_prev = goal_now;
  get_value[i] = angle_data;
  ++i;
  //-----------------------
  //直線補間されてい(曲線保管にする必要あり)
  return deviation / 10;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "wheel_control");
  ros::NodeHandle n;
  ros::Publisher wheel_pub =
    n.advertise<std_msgs::Float32MultiArray>("/motor_speed", 10);
  ros::Subscriber controller_sub = n.subscribe("/cmd_vel", 10, velCallback);
  ros::Subscriber joy_sub = n.subscribe("joy", 10, joyCallback);
  ros::Subscriber gyro_sub = n.subscribe("gyro", 10, gyroCallback);
  ros::Rate loop_rate(1000);

  std_msgs::Float32MultiArray msg;
  msg.data.resize(2);

  while (ros::ok()) {
    float angle_diff = angleFeedback();
    wheel_speed[0] = sin(angle + (M_PI / 4)) * speed;
    wheel_speed[1] = sin(angle - (M_PI / 4)) * speed;
    if (vel.angular.z > 0) {
      wheel_speed[0] = vel.angular.z;
      wheel_speed[1] = -vel.angular.z;
    }
    for (int i = 0; i < 2; ++i) {
      msg.data[i] = wheel_speed[i];
    }
    if(flag_change){
        swap(msg.data[0], msg.data[1]);
    }
    wheel_pub.publish(msg);
    loop_rate.sleep();
    ros::spinOnce();
  }
}
