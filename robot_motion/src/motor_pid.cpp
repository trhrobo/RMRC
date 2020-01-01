#include <cmath>
#include <iostream>
#include <ros.h>

double robot_velocity;
double robot_theta;
double goal_theta;

class PID {
public:
  PID(const double *gain, const double freq);
  void PidUpdate(double goal, double now, double prev);
  double Get();

private:
  void Defferential();
  void Integral();
  void Calcurate();
  double Kp;
  double Ki;
  double Kd;
  double FREQ;
  double goal_value = 0;
  double now_value = 0;
  double prev_value = 0;
  double answer_value = 0;
  double integral_value = 0;
  double defferential_value = 0;
};

PID::PID(const double *gain, const double freq) {
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
  -Kd *defferential_value;
}

double PID::Get() { return answer_value; }

void velocityCallback(const std_msgs::Float64 &msg) { robot_v = msg.data; }

void gyroCallback(const std_msgs::Float64 &msg) { robot_theta = msg.data; }

void thetaCallback(const std_msgs::Float64 &msg) { goal_theta = msg.data; }

// Vx, Vyを定義する必要あり
void wheelCal(double &pwm) {
  double Vx = robot_velocity * cos(robot_theta);
  double Vy = robot_velocity * sin(robot_theta);
  pwm[0] = 0.5 * R * Vy + (L / R) * goal_theta;
  pwm[1] = -0.5 * R * Vx + (L / R) * goal_theta;
}

struct gain {
  double Kp;
  double Ki;
  double Kd;
  gain(double _Kp, _Ki, _Kd) : Kp(_Kp), Ki(_Ki), Kd(_Kd);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "wheel_pid");
  ros::NodeHandle n;
  ros::Publisher pwm_pub = n.advertise<std_msgs::Int16>("wheel_pwm", 10);
  ros::Subscriber velocity_sub = n.subscribe("wheel", 10, velocityCallback);
  ros::Subscriber gyro_sub = n.subscribe("gyro", 10, gyroCallback);
  ros::Subscriber theta_sub = n.subscribe("rotate_goal", 10, thetaCallback);
  vector<double> wheel_pwm{0, 0};
  vector<gain> wheel_gain {
    {1, 1, 1}, { 1, 1, 1 }
  }
  const FREQ = 100;
  const RANGE = 100;
  const MULTIPLICATION = 1;
  PID speed_pid[3] = {PID(wheel_gain[0], FREQ), PID(wheel_gain[1], FREQ),
                      PID(wheel_gain[2], FREQ), PID(wheel_gain[3], FREQ)};
  ros::Rate loop_rate(FREQ);

  while (ros::ok()) {
    //各タイヤへの速度の分解
    wheelCal(wheel_pwm);

    for (int i = 0; i < wheel_pwm.size(); ++i) {
      //速さを求める式[m/s]
      wheel_now[i] =
          ((((double)rotary[i].get() / (RANGE * MULTIPLICATION)) * 101.6) /
           1000);
      wheel_speed[i] = (wheel_now[i] - wheel_prev[i]) / (1 / FREQ);
      speed_pid[i].PidUpdate(wheel_pwm[i], wheel_speed[i], prev_speed[i]);
      //			speed_pid[i].PidUpdate((double)pid_goal,
      // wheel_speed[i], prev_speed[i]);
      pid_result[i] = speed_pid[i].Get();
      wheel_prev[i] = wheel_now[i];
      prev_speed[i] = wheel_speed[i];
      info.data = pid_result[i];
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
