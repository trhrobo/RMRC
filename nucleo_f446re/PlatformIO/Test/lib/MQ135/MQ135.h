/***************************************************
  This is a library for MQ135 gas sensor

  Written by Nerea Gómez.
  
 ****************************************************/

#ifndef MBED_MQ135_H
#define MBED_MQ135_H

 
#include "mbed.h"

class MQ135 {
    public:
        MQ135(PinName pin);
        void initialize();
        float getPPM();
        long getRs();
        float getRsRo();
      
    private:  
        AnalogIn _pin;
        long adc_limit;
        int mqR;
        long rO;
        float a;
        float b; 
        long rS;
        float rSrO;
};
 
#endif
 
 
 
 
 