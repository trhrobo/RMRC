#include"pid.h"
pidCal::pidCal(float user_kp, float user_ki, float user_kd):kp(user_kp), ki(user_ki), kd(user_kd){}
void pidCal::pidMain(){}
float pidCal::pidUpdate(float goal, float now){
    float now_time = _timer.read();
    dt = now_time - time;
    err[0] = goal - now;
    float p = kp * err[0];
    i_sum += goal * dt;
    float i = ki * i_sum;
    float d = kd * (err[0] - err[1]) / dt;
    float order = p + i + d;
    err[1] = err[0];
    time = now_time;
    return order;
}