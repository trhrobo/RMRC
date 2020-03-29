#include<mbed.h>

class pidCal{
    public:
        pidCal(float, float, float);
        void pidMain();
        float pidUpdate(float, float);
    private:
        Timer _timer;
        float kp;
        float ki;
        float kd;
        float dt;
        float err[2];
        float i_sum;
        float time;
};