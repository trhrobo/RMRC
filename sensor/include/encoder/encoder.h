#include<cstdint>
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

