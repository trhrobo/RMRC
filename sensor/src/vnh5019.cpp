#include "vnh5019/vnh5019.h"
#include <pigpiod_if2.h>

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
