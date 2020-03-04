#ifndef VNH5019_H
#define VNH5019_H
#include"mbed.h"

#define THRESHOLD 1.00

typedef struct{
    PinName pin_A;
    PinName pin_B;
    PinName pin_PWM;
    PinName pin_CS;
    PinName pin_EN;
}MotorDriverPin;

class MotorDriver{
    private:
        DigitalOut _pin_A;
        DigitalOut _pin_B;
        DigitalOut _pin_PWM;
        DigitalOut _pin_EN;
        AnalogIn _pin_CS;
    public:
        //MotorDriver(MotorDriverPin user);
        //MotorDriver(PinName A, PinName B, PinName PWM, PinName CS, PinName EN);

        MotorDriver(PinName pin_A, PinName pin_B, PinName pin_PWM, PinName pin_CS, PinName pin_EN):_pin_A(pin_A), _pin_B(pin_B), _pin_CS(pin_CS), _pin_EN(pin_EN){
        /*_pin_A(A);
        _pin_B(B);
        _pin_PWM(PWM);
        _pin_CS(CS);
        _pin_EN(EN);*/
        }
        void setPwm(int pwm);
        bool currentLimit();
};
#endif