#include<ros/ros.h>
#include<pigpiod_if2.h>
#include<dozap/msg/Motor.h>
#include"md.hpp"

int speed_right_front;
int speed_right_back;
int speed_left_front;
int speed_left_back;

void md_callback(const dozap::Plan &point){
    if(point.motor_right > 0){
        motor_right_front(speed_right);
    }else if(point.motor_right < 0){
        motor_right_back(-point.motor_right);
    }else if(point.motor_right == 0){
        motor_right_stop(0);
    }else if(point.motor_left > 0){
        motor_left_front(point.motor_left);
    }else if(point.motor_left < 0){
        motor_left_back(-point.motor_left);
    }else if(point.motor)
}

void motor_right_front(int speed){
    gpio_write(pi, 1, 1);
    gpio_write(pi, 2, 0)
    set_PWM_dutycycle(pi, 1, speed);
}

void motor_right_back(int speed){
    gpio_write(pi, 1, 0);
    gpio_write(pi, 2, 1);
    set_PWM_dutycycle(pi, 1, speed);
}

void motor_right_stop(int speed){
    gpio_write(pi, 1, 0);
    gpio_write(pi, 2, 0);
    set_PWM_dutycycle(pi, 1, speed);
}

void motor_left_front(int speed){
    gpio_write(pi, 1, 1);
    gpio_write(pi, 2, 0);
    set_PWM_dutycycle(pi, 2, speed);
}

void motor_left_back(int speed){
    gpio_write(pi, 1, 0);
    gpio_write(pi, 2, 1);
    set_PWM_dutycycle(pi, 2, speed);
}

void motor_left_stop(int speed){
    gpio_write(pi, 1, 0);
    gpio_write(pi, 2, 0);
    set_PWM_dutycycle(pi, 2, speed);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "md");
    ros::Nodehandle n;
    ros::Subscriber motor_sub = n.subscribe("motor_info", 1000, md_callback);
    ros::Rate loop_rate(10);

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


