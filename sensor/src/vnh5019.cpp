#include <iostream>
#include <pigpiod_if2.h>
#include <ros/ros.h>
#include <std_msgs/Int64MultiArray.h>
using std::abs;

struct mdPin {
  int pin_pwm;
  int pin_change_a;
  int pin_change_b;
};

class VNH5019 {
private:
  int gpio_handle;
  int pin_pwm;
  int pin_change_a;
  int pin_change_b;
  void front(int pwm);
  void back(int pwm);

public:
  VNH5019(mdPin user);
  void set(int pwm);
};

VNH5019::VNH5019(mdPin user) {
  pin_pwm = user.pin_pwm;
  pin_change_a = user.pin_change_a;
  pin_change_b = user.pin_change_b;
  gpio_handle = pigpio_start(0, 0);
  set_mode(gpio_handle, pin_change_a, PI_OUTPUT);
  set_mode(gpio_handle, pin_change_b, PI_OUTPUT);
  set_mode(gpio_handle, pin_pwm, PI_OUTPUT);
}
void VNH5019::front(int pwm) {
  gpio_write(gpio_handle, pin_change_a, 1);
  gpio_write(gpio_handle, pin_change_b, 0);
  set_PWM_dutycycle(gpio_handle, pin_pwm, pwm);
}

void VNH5019::back(int pwm) {
  gpio_write(gpio_handle, pin_change_a, 0);
  gpio_write(gpio_handle, pin_change_b, 1);
  set_PWM_dutycycle(gpio_handle, pin_pwm, pwm);
}

void VNH5019::set(int pwm) {
  if (pwm > 0) {
    front(pwm);
  } else {
    back(-pwm);
  }
}

int pwm_get[2]{};
void pwmCallback(const std_msgs::Int64MultiArray &msg) {
  pwm_get[0] = msg.data[0];
  pwm_get[1] = msg.data[1];
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vnh5019");
  ros::NodeHandle n;
  ros::Subscriber pwm_sub = n.subscribe("wheel_pwm", 10, pwmCallback);
  ros::Rate loop_rate(500);

  mdPin rightPin;
  rightPin.pin_pwm = 1;
  rightPin.pin_change_a = 2;
  rightPin.pin_change_b = 3;

  mdPin leftPin;
  leftPin.pin_pwm = 4;
  leftPin.pin_change_a = 5;
  leftPin.pin_change_b = 6;

  VNH5019 motor_right(rightPin);
  VNH5019 motor_left(leftPin);

  while (ros::ok) {
    motor_right.set(pwm_get[0]);
    motor_left.set(pwm_get[1]);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
