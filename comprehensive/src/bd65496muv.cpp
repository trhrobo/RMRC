#include <iostream>
#include <pigpiod_if2.h>
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
using std::abs;

constexpr int right_pwm_pin = 1;
constexpr int left_pwm_pin = 1;
constexpr int right_change_pin = 1;
constexpr int left_change_pin = 1;
int pwm_right = 0, pwm_left = 0;

class bd65496 {
  private:
    int gpio_handle;
    int pwm_pin, change_pin;
    void front(int pwm);
    void back(int pwm);

  public:
    bd65496(int user_pwm_pin, int user_change_pin);
    void set(int pwm);
};

bd65496::bd65496(int user_pwm_pin, int user_change_pin) {
  pwm_pin = user_change_pin;
  change_pin = user_change_pin;
  gpio_handle = pigpio_start(0, 0);
  set_mode(gpio_handle, change_pin, PI_OUTPUT);
  set_mode(gpio_handle, pwm_pin, PI_OUTPUT);
}

void bd65496::set(int pwm) {
  pwm >= 0 ? this->front(abs(pwm)) : this->back(abs(pwm));
}

void bd65496::front(int pwm) {
  gpio_write(gpio_handle, change_pin, 1);
  set_PWM_dutycycle(gpio_handle, pwm_pin, pwm);
}

void bd65496::back(int pwm) {
  gpio_write(gpio_handle, change_pin, 0);
  set_PWM_dutycycle(gpio_handle, pwm_pin, pwm);
}

void wheelCallback(const std_msgs::Int16MultiArray &msg) {
  pwm_right = msg.data[0];
  pwm_left = msg.data[1];
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "md");
  ros::NodeHandle n;
  ros::Subscriber wheel_sub = n.subscribe("wheel", 10, wheelCallback);
  bd65496 right_wheel(right_pwm_pin, right_change_pin);
  bd65496 left_wheel(left_pwm_pin, left_change_pin);
  ros::Rate loop_rate(1000);
  while (ros::ok()) {
    right_wheel.set(pwm_right);
    left_wheel.set(pwm_left);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
