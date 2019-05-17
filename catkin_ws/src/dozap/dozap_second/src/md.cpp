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
constexpr int RIGHT_DUTY = 255;
constexpr int LEFT_DUTY = 255;
constexpr int RIGHT_ROTATION_1 = 100;
constexpr int RIGHT_ROTATION_2 = 80;
constexpr int LEFT_ROTATION_1 = 100;
constexpr int LEFT_ROTATION_2 = 80;
constexpr int STOP = 0;
int pi;

void motor_right_cw(void);
void motor_right_ccw(void);
void motor_left_cw(void);
void motor_left_ccw(void);
void motor_rotation_foward_right(void);
void motor_rotation_foward_left(void);
void motor_rotation_back_right(void);
void motor_rotation_back_left(void);
void motor_stop(void);

void main_callback(const dozap_second::Main &main_msg){
//いずれは旋回成分を並行成分と合成させる(まだGY521を実装していない)
    if(main_msg.motor_right > 0){motor_right_cw();}
    if(main_msg.motor_right < 0){motor_right_ccw();}
    if(main_msg.motor_right > 0){motor_left_cw();}
    if(main_msg.motor_right < 0){motor_left_ccw();}
    //if(main_msg.motor_right == 0){motor_stop();}
    if(main_msg.rotation_a_right > 0){motor_right_cw();}
    if(main_msg.rotation_b_right > 0){motor_right_cw();}
    if(main_msg.rotation_a_right < 0){motor_right_ccw();}
    if(main_msg.rotation_b_right < 0){motor_right_ccw();}
    if(main_msg.rotation_a_left > 0){motor_left_cw();}
    if(main_msg.rotation_b_left > 0){motor_left_cw();}
    if(main_msg.rotation_a_left < 0){motor_left_ccw();}
    if(main_msg.rotation_b_left < 0){motor_left_ccw();}
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
    set_mode(pi, RIGHT_MOTOR_IN1, PI_OUTPUT);
    set_mode(pi, RIGHT_MOTOR_IN2, PI_OUTPUT);
    set_mode(pi, RIGHT_MOTOR_PWM, PI_OUTPUT);
    set_mode(pi, LEFT_MOTOR_IN1, PI_OUTPUT);
    set_mode(pi, LEFT_MOTOR_IN2, PI_OUTPUT);
    set_mode(pi, LEFT_MOTOR_PWM, PI_OUTPUT);
    set_PWM_frequency(pi, RIGHT_MOTOR_PWM, MOTOR_FREQ);
    set_PWM_frequency(pi, LEFT_MOTOR_PWM, MOTOR_FREQ);


    ros::Rate loop_rate(10);
    
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void motor_right_cw(void){
     gpio_write(pi, RIGHT_MOTOR_IN1, 1);
     gpio_write(pi, RIGHT_MOTOR_IN1, 0);
     set_PWM_dutycycle(pi, RIGHT_MOTOR_PWM, RIGHT_DUTY);
     ROS_INFO("cw");
}
void motor_right_ccw(void){
     gpio_write(pi, RIGHT_MOTOR_IN1, 0);
     gpio_write(pi, RIGHT_MOTOR_IN2, 1);
     set_PWM_dutycycle(pi, RIGHT_MOTOR_PWM, RIGHT_DUTY);
     ROS_INFO("ccw");
}
void motor_left_cw(void){
     gpio_write(pi, LEFT_MOTOR_IN1, 1);
     gpio_write(pi, LEFT_MOTOR_IN1, 0);
     set_PWM_dutycycle(pi, LEFT_MOTOR_PWM, LEFT_DUTY);
     ROS_INFO("cw");
}

void motor_left_ccw(void){
     gpio_write(pi, LEFT_MOTOR_IN1, 0);
     gpio_write(pi, LEFT_MOTOR_IN2, 1);
     set_PWM_dutycycle(pi, LEFT_MOTOR_PWM, LEFT_DUTY);
     ROS_INFO("ccw");
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
   
void motor_stop(void){
     gpio_write(pi, LEFT_MOTOR_IN1, 0);
     gpio_write(pi, LEFT_MOTOR_IN2, 0);
     gpio_write(pi, LEFT_MOTOR_IN1, 0);
     gpio_write(pi, LEFT_MOTOR_IN2, 0);
     set_PWM_dutycycle(pi, RIGHT_MOTOR_PWM, STOP);
     set_PWM_dutycycle(pi, LEFT_MOTOR_PWM, STOP);
     ROS_INFO("stop");
}

*/	
