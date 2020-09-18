#include"mbed.h"
#include"AMT102.h"
#include"VNH5019.h"
#include"vl53l0x.h"
#include"bno055.h"
#include<stdio.h>
#include"serial.h"

#define DEBUG 1
uint8_t address_map[2] = {0x2E, 0x30};
Serial pc(USBTX, USBRX, 115200);
//#if DEBUG
//Serial pc(USBTX, USBRX, 115200);
//#else
//Serial raspi(PB_6, PB_7, 115200);
//#endif

uint16_t speed_right{};
uint16_t speed_left{};
uint8_t pwm_right{};
uint8_t pwm_left{};

int main() {
  I2C i2c(PB_9, PB_8);
  //tof_front, tof_back, gyro_roll, gyro_pitch, gyro_yaw
  float send[5]{};
  //speed, direction
  float receive[2]{};
  BNO055 imu(i2c);
  imu.reset();

  VL53L0X tof[2]{
    VL53L0X(i2c),
    VL53L0X(i2c)
  };
  DigitalInOut xshut[2]{
    DigitalInOut(PA_5),
    DigitalInOut(PA_6)
  };
  for(int i = 0; i < 2; ++i){
    xshut[i].output();
    xshut[i] = 0;
  }
  uint8_t table[2] = {0x2E, 0x30};
  for(int i = 0; i < 2; ++i){
    xshut[i].mode(PullUp);
    xshut[i].input();
    tof[i].init();
    tof[i].setTimeout(500);
    tof[i].setAddress(table[i]);
  }
  for(int i = 0; i < 2; ++i){
    tof[i].startContinuous(0);
    pc.printf("%u\n", tof[i].getAddress());
  }
  //MotorDriver motor_right(D5, D4, PB_2, D12, A1);
//  MotorDriver motor_left(D11, D10, D9, D8, A0);
//  RotaryInc rotary(D15,D14,2 * 50.8 * M_PI,200);
  uint16_t tof_data[2] = {};
  while(1){
    for(int i = 0; i < 2; ++i){
      tof_data[i] = tof[i].readRangeContinuousMillimeters();
      if(tof[i].timeoutOccurred()){
        pc.printf("timeout\n");
        return -1;
      }
    }
    
    imu.setmode(OPERATION_MODE_NDOF);
    imu.get_calib();
    imu.get_angles();
    send[0] = tof_data[0];
    send[1] = tof_data[1];
    send[2] = imu.euler.roll;
    send[3] = imu.euler.pitch;
    send[4] = imu.euler.yaw;
    #if DEBUG
    pc.printf("%u, %u, %f, %f, %f\n", tof_data[0], tof_data[1], send[2], send[3], send[4]);
    #else
    serialSend(send, raspi);
    serialReceive(receive, raspi);
    #endif
    //output motor
    //motor_right.setPwm(pwm_right);
    //motor_right.currentLimit();
    //motor_left.setPwm(pwm_left);
    //motor_left.currentLimit();
  }
}
