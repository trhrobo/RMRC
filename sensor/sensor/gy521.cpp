#include<iostream>
#include<pigpiod_if2.h>
#include<chrono>

using std::cout;
using std::endl;

//constexpr double resolution = 131.0;
int pi;
double degree = 0;
/*typedef enum{
    MPU_WHO_AM_I = 0x75;
    MPU_SET = 0x6B;
    MPU_SIGNAL = 0x3B;
    MPU_ADDRESS = 0x68;
    GYRO_ZOUT_H = 0x47;
    GYRO_ZOUT_L = 0x48;
}RegisterMap;
*/
int main(){
    int pi = pigpio_start(0, 0);
  //  RegisterMap set = MPU_SET;
  //  RegisterMap add = MPU_ADDRESS;
    int handle_datum = i2c_open(pi, 1, 0x68, 0);
    i2c_write_byte_data(pi, handle_datum, 0x6B, 0x00);
    i2c_write_byte_data(pi, handle_datum, 0x1B, 0x00);
    time_sleep(3);
    while(1){
        auto time_start = std::chrono::system_clock::now();
        int16_t gzRaw = i2c_read_byte_data(pi, handle_datum, 0X47) << 8 | i2c_read_byte_data(pi, handle_datum, 0x48);
        double gyro_z = gzRaw / 131.0;
        auto time_end = std::chrono::system_clock::now();
        double time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(time_end-time_start).count();
        degree += gyro_z * (time_diff / 1000);
        cout << degree << endl;
       // cout << time_diff / 1000 << endl;
    }
    i2c_close(pi, handle_datum);
}

