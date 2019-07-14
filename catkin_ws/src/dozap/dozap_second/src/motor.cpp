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
constexpr int FRONT = 1;
constexpr int BACK = -1;
constexpr int STOP = 0;
int pi;
int motor_right, motor_left;

class MD{
    public:
        MD(int pin_A, int pin_B, int pin_pwm);
        void OUTPUT(int check, float pwm);
        int convert(float);
    private:
        int user_A, user_B, user_pwm;
};

MD::MD(int pin_A, int pin_B, int pin_pwm){
    user_A = pin_A;
    user_B = pin_B;
    user_pwm = pin_pwm;
}

void MD::OUTPUT(int check, float pwm){
    //check == -1 back
    //check == 0 stop
    //check == 1 front
    switch (check){
        case -1:
            gpio_write(pi, user_A, 1);
            gpio_write(pi, user_B, 0);
            break;
        case 1:
            gpio_write(pi, user_A, 0);
            gpio_write(pi, user_B, 1);
            break;
        default:
            gpio_write(pi, user_A, 0);
            gpio_write(pi, user_B, 0);
            break;
    }
    set_PWM_dutycycle(pi, user_pwm, pwm);
}

int MD::convert(float x){
    //線形補間
    return (int)((x + 1.0) * 255 - 255);
}

void setup(){
    pi = pigpio_start(0, 0);
    set_mode(pi, RIGHT_MOTOR_IN1, 1);
    set_mode(pi, RIGHT_MOTOR_IN2, 1);
    set_mode(pi, RIGHT_MOTOR_PWM, 1);
    set_mode(pi, LEFT_MOTOR_IN1, 1);
    set_mode(pi, LEFT_MOTOR_IN2, 1);
    set_mode(pi, LEFT_MOTOR_PWM, 1);
    //set_PWM_frequency(pi, RIGHT_MOTOR_PWM, MOTOR_FREQ);
    //set_PWM_frequency(pi, LEFT_MOTOR_PWM, MOTOR_FREQ);
}

float judge_right, judge_left;

void controller_callback(const dozap_second::Main &main_msg){
    judge_right = main_msg.motor_right;
    judge_left = main_msg.motor_left;
}

int main(int argc, char **argv){
    setup();
    ros::init(argc, argv, "md");
    ros::NodeHandle n;
    ros::Subscriber md_sub = n.subscribe("controller_info", 10, controller_callback);
    MD right_motor = MD(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_MOTOR_PWM);
    MD left_motor = MD(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_MOTOR_PWM);
    ros::Rate loop_rate(10);

    while(ros::ok()){
        motor_right = right_motor.convert(judge_right);
        motor_left = left_motor.convert(judge_left);
        //いずれは旋回成分を並行成分と合成させる(まだGY521を実装していない)
        if(motor_right > 0)right_motor.OUTPUT(FRONT, motor_right);
        if(motor_right < 0)right_motor.OUTPUT(BACK, motor_right);
        if(motor_left > 0)left_motor.OUTPUT(FRONT, motor_left);
        if(motor_left < 0)left_motor.OUTPUT(BACK, motor_left);
        if(motor_right == 0 || motor_left == 0)right_motor.OUTPUT(STOP, 0), left_motor.OUTPUT(STOP, 0);
        /*
        if(main_msg.rotation_a_right > 0)right_motor.OUTPUT(FRONT, (float)RIGHT_DUTY);
        if(main_msg.rotation_b_right > 0)right_motor.OUTPUT(FRONT, (float)RIGHT_DUTY);
        if(main_msg.rotation_a_right < 0)right_motor.OUTPUT(BACK, (float)RIGHT_DUTY);
        if(main_msg.rotation_b_right < 0)right_motor.OUTPUT(BACK, (float)RIGHT_DUTY);
        if(main_msg.rotation_a_left > 0)left_motor.OUTPUT(FRONT, (float)LEFT_DUTY);
        if(main_msg.rotation_b_left > 0)left_motor.OUTPUT(FRONT, (float)LEFT_DUTY);
        if(main_msg.rotation_a_left < 0)left_motor.OUTPUT(BACK, (float)LEFT_DUTY);
        if(main_msg.rotation_b_left < 0)left_motor.OUTPUT(BACK, (float)LEFT_DUTY);
        */
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}




