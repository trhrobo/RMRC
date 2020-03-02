#include "MQ135.h"
#include "mbed.h"

MQ135::MQ135(PinName pin) : _pin(pin) {
   AnalogIn co2_analog(pin);
}
void MQ135::initialize(){
    mqR = 22000;
    rO = 41763;
    a = 116.6020682;
    b = -2.769034857;
    adc_limit = 65535;
    }
float MQ135::getPPM() {
    float adcRaw=_pin.read_u16();
    rS = ((adc_limit  * mqR) / adcRaw) - mqR;
    rSrO = (float)rS / (float)rO;
    float ppm = a * pow(rSrO, b);
    return ppm;
}

long MQ135::getRs(){
    return rS;
}
    
float MQ135::getRsRo(){
    return rSrO;
}