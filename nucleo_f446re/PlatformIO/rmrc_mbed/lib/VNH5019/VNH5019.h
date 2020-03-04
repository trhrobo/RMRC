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
    public:
        /*MotorDriver(MotorDriverPin user){
            _pin_A(user.pin_A);
            _pin_B(user.pin_B);
            _pin_PWM(user.pin_PWM);
            _pin_CS(user.pin_CS);
            _pin_EN(user.pin_EN);
        }*/
        //MotorDriver(PinName A, PinName B, PinName PWM, PinName CS, PinName EN);

        /*MotorDriver(PinName pin_A, PinName pin_B, PinName pin_PWM, PinName pin_CS, PinName pin_EN):_pin_A(pin_A), _pin_B(pin_B), _pin_CS(pin_CS), _pin_EN(pin_EN){
        _pin_A(A);
        _pin_B(B);
        _pin_PWM(PWM);
        _pin_CS(CS);
        _pin_EN(EN);
        }*/
        MotorDriver(PinName pin_A, PinName pin_B, PinName pin_PWM, PinName pin_EN, PinName pin_CS) : _pin_A(pin_A), _pin_B(pin_B), _pin_PWM(pin_PWM), _pin_EN(pin_EN), _pin_CS(pin_CS){
            //_pin_A = 0;
            //_pin_B = 0;
            //_pin_PWM = 0;
            //_pin_EN = 0;
        }
        void setPwm(int pwm);
        bool currentLimit();
    private:
        DigitalOut _pin_A;
        DigitalOut _pin_B;
        DigitalOut _pin_PWM;
        DigitalOut _pin_EN;
        AnalogIn _pin_CS;
};
#endif