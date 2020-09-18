#include"mbed.h"
#include"AMT102.h"
#include"VNH5019.h"
#include"vl53l0x.h"
#include"bno055.h"
#include<stdio.h>
#include"serial.h"

#define ADDRESS_DEFALUT 0b0101001 // 0x29
#define ADDRESS_00 (ADDRESS_DEFALUT + 2)
#define DEBUG 1
int address_map[2] = {23, 24};
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
  pc.printf("start");
  I2C i2c(PB_9, PB_8);
  Timer tof_timer;
  //tof_front, tof_back, gyro_roll, gyro_pitch, gyro_yaw
  float send[5]{};
  //speed, direction
  float receive[2]{};
  BNO055 imu(i2c);
  imu.reset();
  VL53L0X tof[2]{
    VL53L0X(i2c, tof_timer),
    VL53L0X(i2c, tof_timer)
  };
  DigitalInOut tof_XSHUT[2]{
    DigitalInOut(PA_5), 
    DigitalInOut(PA_6)
  };
  for(int i = 0; i < 2; ++i){
    tof_XSHUT[i].output();
    tof_XSHUT[i] = 0;
  }
  for(int i = 0; i < 2; ++i){
    tof_XSHUT[i].input();
    if(tof[i].init() == true){
      tof[i].setTimeout(1000);
      uint8_t address = uint8_t(address_map[i]);
      tof[i].setAddress(address);
      #ifdef DEBUG 
      pc.printf("%u\n", address);
      #endif
    }else{
      #ifdef DEBUG
      pc.printf("tof error\n");
      #endif
    }
  }
  for(int i = 0; i < 2; ++i){
    tof[i].startContinuous(0);
  }
  MotorDriver motor_right(D5, D4, PB_2, D12, A1);
//  MotorDriver motor_left(D11, D10, D9, D8, A0);
//  RotaryInc rotary(D15,D14,2 * 50.8 * M_PI,200);
  uint16_t tof_data[2] = {};
  while(1){
    for(int i = 0; i < 2; ++i){
      tof_data[i] = tof[i].readRangeContinuousMillimeters();
      if(tof[i].timeoutOccurred()){
        #if DEBUG
        pc.printf("%u, %u\n", tof[0].getAddress(), tof[1].getAddress());
        pc.printf("timeout\n");
        #endif
        return -1;
      }
    }
    wait_ms(1000);
    imu.setmode(OPERATION_MODE_NDOF);
    imu.get_calib();
    imu.get_angles();
    send[0] = tof_data[0];
    send[1] = tof_data[1];
    send[2] = imu.euler.roll;
    send[3] = imu.euler.pitch;
    send[4] = imu.euler.yaw;
    #if DEBUG
    pc.printf("%u, %u, %f, %f, %f", tof_data[0], tof_data[1], send[2], send[3], send[4]);
    #else
    serialSend(send, raspi);
    serialReceive(receive, raspi);
    #endif
    //output motor
    motor_right.setPwm(pwm_right);
    //motor_right.currentLimit();
    //motor_left.setPwm(pwm_left);
    //motor_left.currentLimit();
  }
}
