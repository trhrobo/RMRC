#include <mbed.h>

DigitalIn dummy(PA_5, PullNone);
AnalogIn co2_analog(PA_5);
Serial send(PB_6, PB_7, 115200);

int map(){
    float data = co2_analog.read() * 5 / 3;
    return static_cast<int>(data * 1024);
    //return 2;
}

int main(){
    int send_value;
    while (1){
        send_value = map();
        send.putc(send_value);
    }
}