#include<ros/ros.h>
#include<pigpiod_if2.h>
#include<vector>
class BNO055{
    BNO055(){

    }
    bool init(){

    }
    bool calib(){

    }
    
};
int main(int argc, char **argv){
    ros::init(argc, argv, "bno");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    ros::Publisher bno_pub = n.advertise<sensor_msgs::Imu>("gyro_info", 10);
    int pi = pigpio_start(0, 0);
    //0x68はby521のやつなので変える必要あり
    int handle_datum = i2c_open(pi, 1, 0x68, 0);
}