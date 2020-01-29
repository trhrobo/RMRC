#include <pigpiod_if2.h>
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int64MultiArray.h>

namespace right {
constexpr int pin_A = 17;
constexpr int pin_B = 27;
constexpr int RANGE = 500;
constexpr int MULTIPLICATION = 1;
constexpr double DIAMETER = 100.0;
}

namespace left {
constexpr int pin_A = 22;
constexpr int pin_B = 23;
constexpr int RANGE = 500;
constexpr int MULTIPLICATION = 1;
constexpr double DIAMETER = 100.0;
}

constexpr int LOOP_RATE = 500;
const char *PIGPIOD_HOST = "localhost";
const char *PIGPIOD_PORT = "8888";

class AMT {
private:
  unsigned int pin_A, pin_B;
  unsigned int callback_A, callback_B;
  bool now_A, now_B;
  int pulse_sum;
  int gpio_handle;
  static void oneMultiplication(int pi, unsigned int gpio, unsigned int edge,
                                uint32_t tick, void *userdata);
  static void twoMultiplication(int pi, unsigned int gpio, unsigned int edge,
                                uint32_t tick, void *userdata);
  static void fourMultiplication(int pi, unsigned int gpio, unsigned int edge,
                                 uint32_t tick, void *userdata);

public:
  AMT(int user_A, int user_B, int user_multiplication);
  virtual ~AMT();
  int get();
};

AMT::AMT(int user_A, int user_B, int user_multiplication) {
  pin_A = user_A;
  pin_B = user_B;
  gpio_handle = pigpio_start(const_cast<char *>(PIGPIOD_HOST),
                             const_cast<char *>(PIGPIOD_PORT));
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

int main(int argc, char **argv) {
  ros::init(argc, argv, "encoder");
  ros::NodeHandle n;
  ros::Publisher encoder_pub =
      n.advertise<std_msgs::Int64MultiArray>("encoder", 10);
  std_msgs::Int64MultiArray msg;
  msg.data.resize(2);
  // ros::Publisher encoder_pub = n.advertise<std_msgs::Int64>("encoder", 10);
  // std_msgs::Int64 msg;
  AMT encoder_right(right::pin_A, right::pin_B, 1);
  AMT encoder_left(left::pin_A, left::pin_B, 1);
  ros::Rate loop_rate(500);
  int now_pulse_right = 0;
  int prev_pulse_right = 0;
  int now_pulse_left = 0;
  int prev_pulse_left = 0;
  while (ros::ok()) {
    now_pulse_right = encoder_right.get();
    now_pulse_left = encoder_left.get();
    msg.data[0] = ((((now_pulse_right - prev_pulse_right) /
                     (right::RANGE * right::MULTIPLICATION))) *
                   right::DIAMETER) /
                  LOOP_RATE;
    msg.data[1] = ((((now_pulse_left - prev_pulse_left) /
                     (left::RANGE * left::MULTIPLICATION))) *
                   left::DIAMETER) /
                  LOOP_RATE;
    // msg.data = encoder_right.get();
    encoder_pub.publish(msg);
    prev_pulse_right = now_pulse_right;
    prev_pulse_left = now_pulse_left;
    ros::spinOnce();
    loop_rate.sleep();
  }
}
