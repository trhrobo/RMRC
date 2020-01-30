#include "encoder/encoder.h"
#include <pigpiod_if2.h>

AMT::AMT(int user_A, int user_B, int user_multiplication) {
  pin_A = user_A;
  pin_B = user_B;
  gpio_handle = pigpio_start(0, 0);
  set_mode(gpio_handle, pin_A, PI_INPUT);
  set_mode(gpio_handle, pin_B, PI_INPUT);
  set_pull_up_down(gpio_handle, pin_A, PI_PUD_UP);
  set_pull_up_down(gpio_handle, pin_B, PI_PUD_UP);
  set_watchdog(gpio_handle, pin_A, 0);
  set_watchdog(gpio_handle, pin_B, 0);
  pulse_sum = 0;
  switch (user_multiplication) {
  case 2:
    callback_A =
        callback_ex(gpio_handle, pin_A, EITHER_EDGE, twoMultiplication, this);
    callback_B =
        callback_ex(gpio_handle, pin_B, EITHER_EDGE, twoMultiplication, this);
    break;
  default:
    callback_A =
        callback_ex(gpio_handle, pin_A, EITHER_EDGE, oneMultiplication, this);
    callback_B =
        callback_ex(gpio_handle, pin_B, EITHER_EDGE, oneMultiplication, this);
    break;
  }
}

AMT::~AMT() {
  callback_cancel(callback_A);
  callback_cancel(callback_B);
}

void AMT::oneMultiplication(int pi, unsigned int gpio, unsigned int edge,
                            uint32_t tick, void *userdata) {
  AMT *regist = (AMT *)userdata;
  if (gpio == regist->pin_A) {
    regist->now_A = edge;
    if (edge) {
      regist->now_B ? ++(regist->pulse_sum) : --(regist->pulse_sum);
    }
  } else if (gpio == regist->pin_B) {
    regist->now_B = edge;
  }
}

void AMT::twoMultiplication(int pi, unsigned int gpio, unsigned int edge,
                            uint32_t tick, void *userdata) {
  AMT *regist = (AMT *)userdata;
  if (gpio == regist->pin_A) {
    regist->now_A = edge;
    if (edge) {
      regist->now_B ? ++(regist->pulse_sum) : --(regist->pulse_sum);
    } else {
      regist->now_B ? --(regist->pulse_sum) : ++(regist->pulse_sum);
    }
  } else if (gpio == regist->pin_B) {
    regist->now_B = edge;
  }
}

int AMT::get() { return pulse_sum; }
