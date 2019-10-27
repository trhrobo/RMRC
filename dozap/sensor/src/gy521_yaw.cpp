#include<iostream>
#include<pigpiod_if2.h>
#include<chrono>
#include<ros/ros.h>
#include<std_msgs/Float32.h>
using std::cout;
using std::endl;
std_msgs::Float32 yaw;
//constexpr double resolution = 131.0;
constexpr double lpf_value = 2.0;
int pi;
double degree = 0;
double lpf_prev = 0;
double lpf_gyro = 0;
/*typedef enum{
  MPU_WHO_AM_I = 0x75;
  MPU_SET = 0x6B;
  MPU_SIGNAL = 0x3B;
  MPU_ADDRESS = 0x68;
  GYRO_ZOUT_H = 0x47;
  GYRO_ZOUT_L = 0x48;
  }RegisterMap;
  */
int main(int argc, char **argv){
    ros::init(argc, argv, "gyro_yaw");
    ros::NodeHandle n;
    ros::Publisher gyro_yaw_pub = n.advertise<std_msgs::Float32>("gyro_yaw", 10);
    int pi = pigpio_start(0, 0);
    //  RegisterMap set = MPU_SET;
    //  RegisterMap add = MPU_ADDRESS;
    int handle_datum = i2c_open(pi, 1, 0x68, 0);
    i2c_write_byte_data(pi, handle_datum, 0x6B, 0x00);
    i2c_write_byte_data(pi, handle_datum, 0x1B, 0x00);
    time_sleep(3.0);
    double loops_number = 100;
    double y_start_first = 0;
    double y_start_second = 0;
    double y_start_third = 0;
    double y_end_first = 0;
    double y_end_second = 0;
    double y_end_third = 0;
    for(int i = 0; i < 100; ++i){
        auto time_start = std::chrono::system_clock::now();
        int16_t gzRaw = i2c_read_byte_data(pi, handle_datum, 0X47) << 8 | i2c_read_byte_data(pi, handle_datum, 0x48);
        double gyro_z = gzRaw / 131.0;
        auto time_end = std::chrono::system_clock::now();
        double time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(time_end-time_start).count();
        degree += gyro_z * (time_diff / 1000);
        switch(i){
            case 0:
                y_start_first = degree;
                break;
            case 1:
                y_start_second = degree;
                break;
            case 2:
                y_start_third = degree;
                break;
            case 97:
                y_end_first = degree;
                break;
            case 98:
                y_end_second = degree;
                break;
            case 99:
                y_end_third = degree;
                break;
        }
    }

    //calibration function
    double y_first = (y_start_first + y_start_second + y_start_third) / 3;
    double y_second = (y_end_first + y_end_second + y_end_third) / 3;
    double function_a = -((y_second - y_first) / 100);

    while(ros::ok()){
        ++loops_number;
        auto time_start = std::chrono::system_clock::now();
        int16_t gzRaw = i2c_read_byte_data(pi, handle_datum, 0X47) << 8 | i2c_read_byte_data(pi, handle_datum, 0x48);
        double gyro_z = gzRaw / 131.0;
        auto time_end = std::chrono::system_clock::now();
        double time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(time_end-time_start).count();
        degree += gyro_z * (time_diff / 1000);
        double z_correction = (degree + (function_a * loops_number));
        //      double lpf_gyro = (1 - lpf_value) * lpf_prev + lpf_value * degree;i
        //        lpf_gyro += lpf_value * (degree - lpf_prev);
        //        lpf_prev = lpf_gyro;
        cout << z_correction << endl;
        yaw.data = (float)degree + ((float)function_a * (float)loops_number);
        //        cout << time_diff << endl;
        gyro_yaw_pub.publish(yaw);
    }
    i2c_close(pi, handle_datum);
}


