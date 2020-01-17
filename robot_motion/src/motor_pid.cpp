#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int64MultiArray.h>
#include <vector>

using std::vector;

double robot_velocity;
double robot_theta;
double goal_theta;
vector<double> goal_vel{0, 0};
vector<int> encoder_now{0, 0};

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

void velocityCallback(const std_msgs::Float64MultiArray &msg) {
  goal_vel[0] = msg.data[0];
  goal_vel[1] = msg.data[1];
}

void encoderCallback(const std_msgs::Int64MultiArray &msg) {
  encoder_now[0] = msg.data[0];
  encoder_now[1] = msg.data[1];
}

struct gain {
  double Kp;
  double Ki;
  double Kd;
  gain(double _Kp, double _Ki, double _Kd) : Kp(_Kp), Ki(_Ki), Kd(_Kd){};
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "wheel_pid");
  ros::NodeHandle n;
  ros::Publisher pwm_pub =
      n.advertise<std_msgs::Int64MultiArray>("wheel_pwm", 10);
  ros::Subscriber velocity_sub = n.subscribe("wheel", 10, velocityCallback);
  ros::Subscriber encoder_sub = n.subscribe("encoder", 10, encoderCallback);
  vector<double> wheel_pwm{0, 0};
  vector<gain> wheel_gain{{1, 1, 1}, {1, 1, 1}};
  constexpr int FREQ = 100;
  constexpr int RANGE = 100;
  constexpr int MULTIPLICATION = 1;
  PID speed_pid[3] = {PID(wheel_gain[0], FREQ), PID(wheel_gain[1], FREQ),
                      PID(wheel_gain[2], FREQ), PID(wheel_gain[3], FREQ)};
  ros::Rate loop_rate(FREQ);

  vector<double> speed_now{0, 0, 0, 0};
  vector<double> speed_prev{0, 0, 0, 0};
  vector<double> wheel_now{0, 0, 0, 0};
  vector<double> wheel_prev{0, 0, 0, 0};
  vector<double> pid_result{0, 0, 0, 0};

  std_msgs::Int64MultiArray info;
  pid_result.data.resize(4);

  while (ros::ok()) {
    //各タイヤへの速度の分解
    for (int i = 0; i < wheel_pwm.size(); ++i) {
      //速さを求める式[m/s]
      wheel_now[i] =
          ((encoder_now[i] / (RANGE * MULTIPLICATION)) * 101.6) / 1000;
      wheel_speed[i] = (wheel_now[i] - wheel_prev[i]) / (1 / FREQ);
      speed_pid[i].PidUpdate(goal_vel[i], speed_now[i], speed_prev[i]);
      //			speed_pid[i].PidUpdate((double)pid_goal,
      // wheel_speed[i], prev_speed[i]);
      pid_result[i] = speed_pid[i].Get();
      wheel_prev[i] = wheel_now[i];
      speed_prev[i] = speed_now[i];
      info.data = pid_result[i];
    }
    pwm_pub.publish(info);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
