#include<iostream>
#include<pigpiod_if2.h>
#include<chrono>
#include<sensor_msgs/Imu.h>
#include<ros/ros.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/Quaternion.h>
#include<tf/transform_broadcaster.h>

using std::cout;
using std::endl;

//constexpr double resolution = 131.0;
int pi;
double degree_x = 0;
double degree_y = 0;
double degree_z = 0;
double lpf_prev = 0;
double lpf_gyro = 0;
double loops_number = 100;
/*typedef enum{
  MPU_WHO_AM_I = 0x75;
  MPU_SET = 0x6B;
  MPU_SIGNAL = 0x3B;
  MPU_ADDRESS = 0x68;
  GYRO_ZOUT_H = 0x47;
  GYRO_ZOUT_L = 0x48;
  }RegisterMap;
  */
geometry_msgs::Quaternion rpy_to_geometry_quat(double roll, double pitch, double yaw);
double calibration(int address_h, int address_l, int handle_datum);

int main(int argc, char **argv){
    ros::init(argc, argv, "gyro");
    ros::NodeHandle n;
    ros::Publisher gyro_pub = n.advertise<sensor_msgs::Imu>("gyro_info", 10);
    int pi = pigpio_start(0, 0);
    //  RegisterMap set = MPU_SET;
    //  RegisterMap add = MPU_ADDRESS;
    int handle_datum = i2c_open(pi, 1, 0x68, 0);
    i2c_write_byte_data(pi, handle_datum, 0x6B, 0x00);
    i2c_write_byte_data(pi, handle_datum, 0x1B, 0x00);
    i2c_write_byte_data(pi, handle_datum, 0x1c, 0x00);
    time_sleep(3.0);

    int a = 0x43, b = 0x44;
    double function_x = calibration(a, b, handle_datum);
    a = 0x45, b = 0x46;
    double function_y = calibration(a, b, handle_datum);
    a = 0x47, b = 0x48;
    double function_z = calibration(a, b, handle_datum);

    while(ros::ok()){
        ++loops_number;
        auto time_start = std::chrono::system_clock::now();

        int16_t axRaw = i2c_read_byte_data(pi, handle_datum, 0X3B) << 8 | i2c_read_byte_data(pi, handle_datum, 0x3C);
        int16_t ayRaw = i2c_read_byte_data(pi, handle_datum, 0X3D) << 8 | i2c_read_byte_data(pi, handle_datum, 0x3E);
        int16_t azRaw = i2c_read_byte_data(pi, handle_datum, 0X3F) << 8 | i2c_read_byte_data(pi, handle_datum, 0x40);

        double accel_x = axRaw / 16384;
        double accel_y = ayRaw / 16384;
        double accel_z = azRaw / 16384;

//        imu_msg.header.frame_id = "map";
//        imu_msg.header.stamp = ros::Time::now();
//        imu.linear_acceleration_covariance.x = accel_x;
//        imu.linear_acceleration_covariance.y = accel_y;
//        imu.linear_acceleration_covariance.z = accel_z;

        int16_t gxRaw = i2c_read_byte_data(pi, handle_datum, 0X43) << 8 | i2c_read_byte_data(pi, handle_datum, 0x44);
        int16_t gyRaw = i2c_read_byte_data(pi, handle_datum, 0X45) << 8 | i2c_read_byte_data(pi, handle_datum, 0x46);
        int16_t gzRaw = i2c_read_byte_data(pi, handle_datum, 0X47) << 8 | i2c_read_byte_data(pi, handle_datum, 0x48);

        double gyro_x = gxRaw / 131.0;
        double gyro_y = gyRaw / 131.0;
        double gyro_z = gzRaw / 131.0;

        //imu.angular_velocity_covariance.x = gyro_x;
        //imu.angular_velocity_covariance.y = gyro_y;
        //imu.angular_velocity_covariance.z = gyro_z;

        //gyro_pub.publish(imu);


        auto time_end = std::chrono::system_clock::now();
        double time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(time_end-time_start).count();
        degree_x += gyro_x * (time_diff / 1000);
        degree_y += gyro_y * (time_diff / 1000);
        degree_z += gyro_z * (time_diff / 1000);
        //      double lpf_gyro = (1 - lpf_value) * lpf_prev + lpf_value * degree;i
        //        lpf_gyro += lpf_value * (degree - lpf_prev);
        //        lpf_prev = lpf_gyro;
        //        cout << time_diff << endl;
        // cout << time_diff / 1000 << endl;
        double degree_x_correction = degree_x + (function_x * loops_number);
        double degree_y_correction = degree_y + (function_y * loops_number);
        double degree_z_correction = degree_z + (function_z * loops_number);
        cout << degree_x_correction << ", " << degree_y_correction << ", " << degree_z_correction << endl;
        cout << degree_z_correction << endl;
        geometry_msgs::Quaternion gyro_data;

        //オイラー角をクォータニオンに変換
        gyro_data = rpy_to_geometry_quat(degree_x_correction, degree_y_correction, degree_z_correction);

        sensor_msgs::Imu imu_msg;
        imu_msg.header.frame_id = "map";
        imu_msg.header.stamp=ros::Time::now();
        imu_msg.orientation.x = gyro_data.x;
        imu_msg.orientation.y = gyro_data.y;
        imu_msg.orientation.z = gyro_data.z;
        imu_msg.orientation.w = gyro_data.w;
        gyro_pub.publish(imu_msg);
    }
    i2c_close(pi, handle_datum);
}


geometry_msgs::Quaternion rpy_to_geometry_quat(double roll, double pitch, double yaw){
    tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    return geometry_quat;
}

double calibration(int address_h, int address_l, int handle_datum){
    double degree_calibration = 0;
    double y_start_first = 0;
    double y_start_second = 0;
    double y_start_third = 0;
    double y_end_first = 0;
    double y_end_second = 0;
    double y_end_third = 0;
    for(int i = 0; i < 100; ++i){
        auto time_start = std::chrono::system_clock::now();
        int16_t calibrationRaw = i2c_read_byte_data(pi, handle_datum, address_h) << 8 | i2c_read_byte_data(pi, handle_datum, address_l);
        double gyro = calibrationRaw / 131.0;
        auto time_end = std::chrono::system_clock::now();
        double time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(time_end-time_start).count();
        degree_calibration += gyro * (time_diff / 1000);
        switch(i){
            case 0:
                y_start_first = degree_calibration;
                break;
            case 1:
                y_start_second = degree_calibration;
                break;
            case 2:
                y_start_third = degree_calibration;
                break;
            case 97:
                y_end_first = degree_calibration;
                break;
            case 98:
                y_end_second = degree_calibration;
                break;
            case 99:
                y_end_third = degree_calibration;
                break;
        }
    }
    //calibration function
    double y_first = (y_start_first + y_start_second + y_start_third) / 3;
    double y_second = (y_end_first + y_end_second + y_end_third) / 3;
    double function_a = -((y_second - y_first) / 100);
    return function_a;
}
