#include<ros/ros.h>
#include<pigpiod_if2.h>
#include<dozap_second/Main.h>
#include"md.hpp"

using ros::MD;

constexpr int RIGHT_MOTOR_IN1 = 2;
constexpr int RIGHT_MOTOR_IN2 = 3;
constexpr int RIGHT_MOTOR_PWM = 4;
constexpr int LEFT_MOTOR_IN1 = 14;
constexpr int LEFT_MOTOR_IN2 = 15;
constexpr int LEFT_MOTOR_PWM = 18;
constexpr int MOTOR_FREQ = 125;
constexpr int RIGHT_DUTY = 100;
constexpr int LEFT_DUTY = 100;
constexpr int RIGHT_ROTATION_1 = 100;
constexpr int RIGHT_ROTATION_2 = 80;
constexpr int LEFT_ROTATION_1 = 100;
constexpr int LEFT_ROTATION_2 = 80;
constexpr int FRONT = 1;
constexpr int BACK = -1;
constexpr int STOP = 0;
int pi;

void motor_rotation_foward_right(void);
void motor_rotation_foward_left(void);
void motor_rotation_back_right(void);
void motor_rotation_back_left(void);
void motor_stop(void);
//自作のマップ関数
int convert(float);
void main_callback(const dozap_second::Main &main_msg){
    int motor_right = convert(main_msg.motor_right);
    int motor_left = convert(main_msg.motor_left);
//    MD right_motor = MD(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_MOTOR_PWM);
    MD right_motor = MD(2, 3, 4);
    MD left_motor = MD(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_MOTOR_PWM);
//いずれは旋回成分を並行成分と合成させる(まだGY521を実装していない)
    if(main_msg.motor_right > 0)right_motor.OUTPUT(FRONT, main_msg.motor_right);
    if(main_msg.motor_right < 0)right_motor.OUTPUT(BACK, main_msg.motor_right);
    if(main_msg.motor_left > 0)left_motor.OUTPUT(FRONT, main_msg.motor_left);
    if(main_msg.motor_left < 0)left_motor.OUTPUT(BACK, main_msg.motor_left);
    if(main_msg.motor_right == 0 || motor_left == 0)right_motor.OUTPUT(STOP, 0), left_motor.OUTPUT(STOP, 0);
    if(main_msg.rotation_a_right > 0)right_motor.OUTPUT(FRONT, (float)RIGHT_DUTY);
    if(main_msg.rotation_b_right > 0)right_motor.OUTPUT(FRONT, (float)RIGHT_DUTY);
    if(main_msg.rotation_a_right < 0)right_motor.OUTPUT(BACK, (float)RIGHT_DUTY);
    if(main_msg.rotation_b_right < 0)right_motor.OUTPUT(BACK, (float)RIGHT_DUTY);
    if(main_msg.rotation_a_left > 0)left_motor.OUTPUT(FRONT, (float)LEFT_DUTY);
    if(main_msg.rotation_b_left > 0)left_motor.OUTPUT(FRONT, (float)LEFT_DUTY);
    if(main_msg.rotation_a_left < 0)left_motor.OUTPUT(BACK, (float)LEFT_DUTY);
    if(main_msg.rotation_b_left < 0)left_motor.OUTPUT(BACK, (float)LEFT_DUTY);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "md");
    ros::NodeHandle n;
    ros::Subscriber md_sub = n.subscribe("pwm_info", 10, main_callback);
//    pi = pigpio_start(0, 0);
    set_mode(pi, RIGHT_MOTOR_IN1, 1);
    set_mode(pi, RIGHT_MOTOR_IN2, 1);
    set_mode(pi, RIGHT_MOTOR_PWM, 1);
    set_mode(pi, LEFT_MOTOR_IN1, 1);
    set_mode(pi, LEFT_MOTOR_IN2, 1);
    set_mode(pi, LEFT_MOTOR_PWM, 1);
//    set_PWM_frequency(pi, RIGHT_MOTOR_PWM, MOTOR_FREQ);
//    set_PWM_frequency(pi, LEFT_MOTOR_PWM, MOTOR_FREQ);


    ros::Rate loop_rate(10);

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

int convert(float x) {
//線形補間
     return (int)((x + 1.0) * 255 -255);
}



