#include <pigpiod_if2.h>
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int64MultiArray.h>

constexpr int right_A = 17;
constexpr int right_B = 27;
constexpr int left_A = 22;
constexpr int left_B = 23;
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
  AMT encoder_right(right_A, right_B, 1);
  AMT encoder_left(left_A, left_B, 1);
  ros::Rate loop_rate(1000);

  while (ros::ok()) {
    msg.data[0] = encoder_right.get();
    msg.data[1] = encoder_left.get();
    // msg.data = encoder_right.get();
    encoder_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
