#include<pigpiod_if2.h>
namespace ros{
class MD{
public:
    MD(int pin_A, int pin_B, int pin_pwm);
    void OUTPUT(int check, float pwm);
private:
    int user_A, user_B, user_pwm;
};
}
