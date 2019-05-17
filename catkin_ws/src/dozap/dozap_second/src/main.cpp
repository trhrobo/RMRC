#include<ros/ros.h>
#include<dozap_second/Button.h>
#include<dozap_second/Main.h>

constexpr int Speed_max = 100;
constexpr int Speed_min = -100;
constexpr int Speed_zero = 0;

dozap_second::Main data;

void controller_callback(const dozap_second::Button& button_msg){
    switch(button_msg.move){
        case 1:
            data.motor_right = Speed_max;
            data.motor_left = Speed_max;
            break;
        case -1:
            data.motor_right = Speed_min;
            data.motor_left = Speed_min;
            break;
        case 0:
            data.motor_right = Speed_zero;
            data.motor_left = Speed_zero;
            break;
    }

    switch(button_msg.rotation_right){
        case 1:
            data.rotation_right = Speed_min;
            data.rotation_left = Speed_max;
            break;
        case -1:
            data.rotation_right = Speed_zero;
            data.rotation_left = Speed_zero;
            break;
        case 0:
            data.rotation_right = Speed_zero;
            data.rotation_left = Speed_zero;
            break;
    }

    switch(button_msg.rotation_left){
        case 1:
            data.rotation_right = Speed_max;
            data.rotation_left = Speed_min;
            break;
        case -1:
            data.rotation_right = Speed_zero;
            data.rotation_left = Speed_zero;
            break;
        case 0:
            data.rotation_right = Speed_zero;
            data.rotation_left = Speed_zero;
            break;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "main");
    ros::NodeHandle n;

    ros::Publisher main_pub = n.advertise<dozap_second::Main>("pwm_info", 10);
    ros::Subscriber main_sub = n.subscribe("controller_info", 10, controller_callback);

    ros::Rate loop_rate(10);

    while(ros::ok()){
        main_pub.publish(data);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}



