/**
 * @file pid.cpp

 * @brief pid実装
**/
#include "pid/pid.h"
#include <cstdlib>
#include <iostream>

PID::PID(double *gain, int freq) {
  Kp = gain[0];
  Ki = gain[1];
  Kd = gain[2];
  FREQ = freq;
}

void PID::PidUpdate(double goal, double now, double prev) {
  goal_value = goal;
  now_value = now;
  prev_value = prev;

  Defferential();
  Integral();
  Calcurate();
}

void PID::Defferential() {
  defferential_value = (now_value - prev_value) / (1 / FREQ);
}

void PID::Integral() { integral_value += now_value * (1 / FREQ); }

void PID::Calcurate() {
  //	cout << goal_value << ":" << now_value << endl;
  answer_value = Kp * (goal_value - now_value);
  //-Kd *defferential_value;
}

double PID::Get() {
  if (abs(answer_value) > 255) {
    answer_value = 255;
  }
  return answer_value;
}
