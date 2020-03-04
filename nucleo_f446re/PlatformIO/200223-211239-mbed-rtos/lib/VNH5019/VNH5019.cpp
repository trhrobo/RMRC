#include"VNH5019.h"
/*
MotorDriver::MotorDriver(MotorDriverPin user){
        _pin_A(user.pin_A);
        _pin_B(user.pin_B);
        _pin_PWM(user.pin_PWM);
        _pin_CS(user.pin_CS);
        _pin_EN(user.pin_EN);
}*/
/*
MotorDriver::MotorDriver(PinName A, PinName B, PinName PWM, PinName CS, PinName EN):_pin_A(A), _pin_B(B), _pin_CS(CS), _pin_EN(EN){
    /*_pin_A(A);
    _pin_B(B);
    _pin_PWM(PWM);
    _pin_CS(CS);
    _pin_EN(EN);
}*/

void MotorDriver::setPwm(int pwm){
    if(pwm > 0){
        _pin_A = 1;
        _pin_B = 0;
        _pin_PWM = (float)pwm / 255;
    }else{
        _pin_A = 0;
        _pin_B = 1;
        _pin_PWM = (float)pwm/-255;
    }
};

bool MotorDriver::currentLimit(){
    if(_pin_CS > THRESHOLD){
        _pin_EN = 0;
        return false;
    }else{
        _pin_EN = 1;
        return true;
    }
}