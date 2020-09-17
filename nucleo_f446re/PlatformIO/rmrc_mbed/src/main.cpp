#include"mbed.h"
#include"AMT102.h"
//#include"x_nucleo_53l0a1.h"
#include"VNH5019.h"
#include"vl53l0x.h"
#include<stdio.h>
#include"rtos.h"
#include<cstring>
#include<cstdint>
#include"serial.h"

#define ADDRESS_DEFALUT 0b0101001 // 0x29
#define ADDRESS_00 (ADDRESS_DEFALUT + 2)
#define DEBUG 0
//BufferedSerial raspi(PB_6, PB_7, 115200);

#if DEBUG
Serial pc(USBTX, USBRX, 115200);
#else
Serial raspi(PB_6, PB_7, 115200);
#endif
//static X_NUCLEO_53L0A1 *board=NULL;

uint16_t speed_right{};
uint16_t speed_left{};
uint8_t pwm_right{};
uint8_t pwm_left{};

int main() {
  I2C i2c(PB_9, PB_8);
  Timer tof_timer;
  //tof_front, tof_back, gyro_roll, gyro_pitch, gyro_yaw
  float send[5]{};
  //speed, direction
  float receive[2]{};
 
  VL53L0X tof[2]{
    VL53L0X(i2c, tof_timer),
    VL53L0X(i2c, tof_timer)
  };
  DigitalInOut tof_XSHUT[2]{D1, D2};

  for(int i = 0; i < 2; ++i){
    tof_XSHUT[i].output();
    tof_XSHUT[i] = 0;
  }

  for(int i = 0; i < 2; ++i){
    tof_XSHUT[i].input();
    tof[i].init();
    tof[i].setTimeout(500);
    tof[i].startContinuous(0);
    int address = ADDRESS_00 + (i * 2);
    tof[i].setAddress(address);
  }
  MotorDriver motor_right(D15, D14, D13, D12, A1);
  MotorDriver motor_left(D11, D10, D9, D8, A0);
  RotaryInc rotary(D15,D14,2 * 50.8 * M_PI,200);
  uint16_t tof_data[2] = {};
  float gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;
  while(1){
    for(int i = 0; i < 2; ++i){
      tof_data[i] = tof[i].readRangeContinuousMillimeters();
      if(tof[i].timeoutOccurred()){
        return -1;
      }
    }
    send[0] = tof_data[0];
    send[1] = tof_data[1];
    send[2] = gyro_roll;
    send[3] = gyro_pitch;
    send[4] = gyro_yaw;
    serialSend(send, raspi);
    serialReceive(receive, raspi);
    //output motor
    motor_right.setPwm(pwm_right);
    motor_right.currentLimit();
    motor_left.setPwm(pwm_left);
    motor_left.currentLimit();
  }
}
