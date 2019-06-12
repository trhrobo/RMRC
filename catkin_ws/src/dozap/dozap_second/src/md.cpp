#include<ros/ros.h>
#include<pigpiod_if2.h>
#include<dozap_second/Main.h>

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
constexpr int STOP = 0;
int pi;

void motor_right_cw(int);
void motor_right_ccw(int);
void motor_left_cw(int);
void motor_left_ccw(int);
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
//いずれは旋回成分を並行成分と合成させる(まだGY521を実装していない)
    if(motor_right > 0){motor_right_cw(motor_right);}
    if(motor_right < 0){motor_right_ccw(-motor_right);}
    if(motor_left > 0){motor_left_cw(motor_left);}
    if(motor_left < 0){motor_left_ccw(-motor_left);}
    if(motor_right == 0 || motor_left == 0){motor_stop();}
    if(main_msg.rotation_a_right > 0){motor_right_cw((float)RIGHT_DUTY);}
    if(main_msg.rotation_b_right > 0){motor_right_cw((float)RIGHT_DUTY);}
    if(main_msg.rotation_a_right < 0){motor_right_ccw((float)RIGHT_DUTY);}
    if(main_msg.rotation_b_right < 0){motor_right_ccw((float)RIGHT_DUTY);}
    if(main_msg.rotation_a_left > 0){motor_left_cw((float)LEFT_DUTY);}
    if(main_msg.rotation_b_left > 0){motor_left_cw((float)LEFT_DUTY);}
    if(main_msg.rotation_a_left < 0){motor_left_ccw((float)LEFT_DUTY);}
    if(main_msg.rotation_b_left < 0){motor_left_ccw((float)LEFT_DUTY);}
//rotation_fowardはGY521を実装してから
 /*   if(main_msg.rotation_a_right < 0 && main_msg.rotation_a_left > 0 && main_msg.motor_right > 0 && main_msg.motor_left > 0){motor_rotation_foward_right();}
    if(main_msg.rotation_b_right < 0 && main_msg.rotation_b_left > 0 && main_msg.motor_right > 0 && main_msg.motor_left > 0){motor_rotation_foward_right();}
    if(main_msg.rotation_a_right > 0 && main_msg.rotation_a_left < 0 && main_msg.motor_right > 0 && main_msg.motor_left > 0){motor_rotation_foward_left();}
    if(main_msg.rotation_b_right > 0 && main_msg.rotation_b_left < 0 && main_msg.motor_right > 0 && main_msg.motor_left > 0){motor_rotation_foward_left();}
    if(main_msg.rotation_a_right < 0 && main_msg.rotation_a_left > 0 && main_msg.motor_right < 0 && main_msg.motor_left < 0){motor_rotation_back_right();}
    if(main_msg.rotation_b_right < 0 && main_msg.rotation_b_left > 0 && main_msg.motor_right < 0 && main_msg.motor_left < 0){motor_rotation_back_right();}
    if(main_msg.rotation_a_right > 0 && main_msg.rotation_a_left < 0 && main_msg.motor_right < 0 && main_msg.motor_left < 0){motor_rotation_back_left();}
    if(main_msg.rotation_b_right > 0 && main_msg.rotation_b_left < 0 && main_msg.motor_right < 0 && main_msg.motor_left < 0){motor_rotation_back_left();}
    //if(main_msg.rotation_right == 0){motor_stop();}
    //if(main_msg.rotation_a_right == 0 && main_msg.rotation_b_right && main_msg.rotation_a_left == 0 && main_msg.rotation_b_left && main_msg.motor_right == 0 && main_msg.motor_left == 0){motor_stop();}
*/    
}

int main(int argc, char **argv){
    ros::init(argc, argv, "md");
    ros::NodeHandle n;
    ros::Subscriber md_sub = n.subscribe("pwm_info", 10, main_callback);
    pi = pigpio_start(0, 0);
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

void motor_right_cw(int value){
     gpio_write(pi, RIGHT_MOTOR_IN1, 1);
     gpio_write(pi, RIGHT_MOTOR_IN2, 0);
     set_PWM_dutycycle(pi, RIGHT_MOTOR_PWM, value);
     ROS_INFO("cw:%d", value);
}
void motor_right_ccw(int value){
     gpio_write(pi, RIGHT_MOTOR_IN1, 0);
     gpio_write(pi, RIGHT_MOTOR_IN2, 1);
     set_PWM_dutycycle(pi, RIGHT_MOTOR_PWM, value);
     ROS_INFO("ccw:%d", value);
}
void motor_left_cw(int value){
     gpio_write(pi, LEFT_MOTOR_IN1, 1);
     gpio_write(pi, LEFT_MOTOR_IN2, 0);
     set_PWM_dutycycle(pi, LEFT_MOTOR_PWM, value);
     ROS_INFO("cw:%d", value);
}

void motor_left_ccw(int value){
     gpio_write(pi, LEFT_MOTOR_IN1, 0);
     gpio_write(pi, LEFT_MOTOR_IN2, 1);
     set_PWM_dutycycle(pi, LEFT_MOTOR_PWM, value);
     ROS_INFO("ccw:%d", value);
}
/*
void motor_rotation_foward_right(void){
     gpio_write(pi, RIGHT_MOTOR_IN1, 1);
     gpio_write(pi, RIGHT_MOTOR_IN1, 0);
     set_PWM_dutycycle(pi, LEFT_MOTOR_PWM, RIGHT_ROTATION_2);
     gpio_write(pi, RIGHT_MOTOR_IN1, 1);
     gpio_write(pi, RIGHT_MOTOR_IN1, 0);
     set_PWM_dutycycle(pi, LEFT_MOTOR_PWM, LEFT_ROTATION_1);
     ROS_INFO("foward_right");
}

void motor_rotation_foward_left(void){
     gpio_write(pi, RIGHT_MOTOR_IN1, 1);
     gpio_write(pi, RIGHT_MOTOR_IN1, 0);
     set_PWM_dutycycle(pi, LEFT_MOTOR_PWM, RIGHT_ROTATION_1);
     gpio_write(pi, RIGHT_MOTOR_IN1, 1);
     gpio_write(pi, RIGHT_MOTOR_IN1, 0);
     set_PWM_dutycycle(pi, LEFT_MOTOR_PWM, LEFT_ROTATION_2);
     ROS_INFO("foward_left");
}

void motor_rotation_back_right(void){
     gpio_write(pi, RIGHT_MOTOR_IN1, 0);
     gpio_write(pi, RIGHT_MOTOR_IN1, 1);
     set_PWM_dutycycle(pi, LEFT_MOTOR_PWM, RIGHT_ROTATION_1);
     gpio_write(pi, RIGHT_MOTOR_IN1, 0);
     gpio_write(pi, RIGHT_MOTOR_IN1, 1);
     set_PWM_dutycycle(pi, LEFT_MOTOR_PWM, LEFT_ROTATION_2);
     ROS_INFO("back_right");
}

void motor_rotation_back_left(void){
     gpio_write(pi, RIGHT_MOTOR_IN1, 0);
     gpio_write(pi, RIGHT_MOTOR_IN1, 1);
     set_PWM_dutycycle(pi, LEFT_MOTOR_PWM, RIGHT_ROTATION_2);
     gpio_write(pi, RIGHT_MOTOR_IN1, 0);
     gpio_write(pi, RIGHT_MOTOR_IN1, 1);
     set_PWM_dutycycle(pi, LEFT_MOTOR_PWM, LEFT_ROTATION_1);
     ROS_INFO("back_left");
}
*/   
void motor_stop(void){
     gpio_write(pi, LEFT_MOTOR_IN1, 0);
     gpio_write(pi, LEFT_MOTOR_IN2, 0);
     gpio_write(pi, LEFT_MOTOR_IN1, 0);
     gpio_write(pi, LEFT_MOTOR_IN2, 0);
     set_PWM_dutycycle(pi, RIGHT_MOTOR_PWM, STOP);
     set_PWM_dutycycle(pi, LEFT_MOTOR_PWM, STOP);
     ROS_INFO("stop");
}

	
