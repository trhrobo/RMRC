#include "mbed.h"
#include "AMT102.h"
#include "VNH5019.h"
#include "VL53L0X.h"
#include "serial.h"
#include "BNO055.h"
#include "pid.h"
#include <stdio.h>
#include "rtos.h"
#include <cstring>
#include <cstdint>
#include <array>
#define DEBUG 0
#define float32_t float
#define range1_addr (0x56)
#define range2_addr (0x60)
#define range3_addr (0x6A)
#define range4_addr (0x74)
#define range1_XSHUT D7
#define range2_XSHUT D8
#define range3_XSHUT D2
#define range4_XSHUT D3
#define SDA D14
#define SCL D15

using std::array;
static DevI2C devI2c(SDA, SCL);
int main()
{
  array<float32_t, 5> send_data;
  array<float32_t, 2> receive_data;
  //create Instance
  BNO055 bno(&devI2c);
  MotorDriver motor_right(D15, D14, D13, D12, A1);
  MotorDriver motor_left(D11, D10, D9, D8, A0);
  RotaryInc rotary_right(D15, D14, 2 * 50.8 * M_PI, 200);
  RotaryInc rotary_left(D0, D1, 2 * 50.8 * M_PI, 200);
  pidCal pid_right_motor(1, 1, 1);
  pidCal pid_left_motor(1, 1, 1);
  SerialRaspi serial(PB_6, PB_7);
  static DigitalOut shutdown1_pin(range1_XSHUT);
  static VL53L0X range1(&devI2c, &shutdown1_pin, NC);
  static DigitalOut shutdown2_pin(range2_XSHUT);
  static VL53L0X range2(&devI2c, &shutdown2_pin, NC);
  static DigitalOut shutdown3_pin(range3_XSHUT);
  static VL53L0X range3(&devI2c, &shutdown3_pin, NC);
  static DigitalOut shutdown4_pin(range4_XSHUT);
  static VL53L0X range4(&devI2c, &shutdown4_pin, NC);
  /*Initial all sensors*/
  range1.init_sensor(range1_addr);
  range2.init_sensor(range2_addr);
  range3.init_sensor(range3_addr);
  range4.init_sensor(range4_addr);

  /*Get datas*/
  uint32_t distance1;
  uint32_t distance2;
  uint32_t distance3;
  uint32_t distance4;

  int status1;
  int status2;
  int status3;
  int status4;
  float gyro;
  while (1){
    bno.setmode(OPERATION_MODE_IMUPLUS);
    bno.get_angles();
    gyro = bno.euler.yaw;
    status1 = range1.get_distance(&distance1);
    status2 = range2.get_distance(&distance2);
    status3 = range3.get_distance(&distance3);
    status4 = range4.get_distance(&distance4);
    send_data[0] = distance1;
    send_data[1] = distance2;
    send_data[2] = distance3;
    send_data[3] = distance4;
    send_data[4] = gyro;
    long long speed_right = rotary_right.getSpeed();
    long long speed_left = rotary_left.getSpeed();
    float pid_right = pid_right_motor.pidUpdate(receive_data[0], speed_right);
    float pid_left = pid_left_motor.pidUpdate(receive_data[1], speed_left);
    //output motor
    motor_right.mdMain(pid_right);
    motor_left.mdMain(pid_left);
    serial.serialMain(send_data, receive_data);
  }
}

