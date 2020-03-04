#include "mbed.h"

#define THRESHOLD 1.00

class MotorDriver{
    /*public:
        MotorDriver(PinName pin_A, PinName pin_B, PinName pin_PWM, PinName pin_CS, PinName pin_EN):_pin_A(pin_A), _pin_B(pin_B), _pin_CS(pin_CS), _pin_EN(pin_EN){
            pin_A = 0;
            pin_B = 0;
            pin_PWM = 0;
            pin_CS = 0;
            pin_EN = 0;
    }*/
    public:
        MotorDriver(PinName pin_A, PinName pin_B, PinName pin_PWM, PinName pin_EN, PinName pin_CS) : _pin_A(pin_A), _pin_B(pin_B), _pin_PWM(pin_PWM), _pin_EN(pin_EN), _pin_CS(pin_CS){
            //_pin_A = 0;
            //_pin_B = 0;
            //_pin_PWM = 0;
            //_pin_EN = 0;
        }
    private:
        DigitalOut _pin_A;
        DigitalOut _pin_B;
        DigitalOut _pin_PWM;
        DigitalOut _pin_EN;
        AnalogIn _pin_CS;
        //DigitalOut _pin_B;
        //DigitalOut _pin_PWM;
        //DigitalOut _pin_EN;
        //AnalogIn _pin_CS;
};

class Flasher {
public:
    Flasher(PinName pin) : _pin(pin) { // デジタル出力の実体化
        _pin = 0; // デジタル出力の初期化
    }
 
    void flash(int n) {
        for(int i=0; i<n*2; n++) {
            _pin = !_pin;
            wait(0.2);
        }
    }
 
private:
    DigitalOut _pin; // デジタル出力の宣言
};
 
 
Flasher led(LED2);
 
int main() {
    led.flash(5);
    led.flash(2);
}
/*#include"mbed.h"

DigitalOut led1(A0);
DigitalOut led2(A1);
Thread thread;

void led2_thread(){
    while(1){
        led2 = !led2;
        wait(1);
    }
}

int main(){
    thread.start(led2_thread);
    while(1){
        led1 = !led1;
        wait(0.5);
    }
    thread.join();
}*/