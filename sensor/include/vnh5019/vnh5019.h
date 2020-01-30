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
