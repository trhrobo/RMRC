#include"mbed.h"
#include"AMT102.h"
#include"x_nucleo_53l0a1.h"
#include"VNH5019.h"
#include<stdio.h>
#include"rtos.h"
#include<cstring>
#include<cstdint>
#include"serial.h"

#define DEBUG 0
#define VL53L0_I2C_SDA   D14 
#define VL53L0_I2C_SCL   D15 

//BufferedSerial raspi(PB_6, PB_7, 115200);

#if DEBUG
Serial pc(USBTX, USBRX, 115200);
#else
Serial raspi(PB_6, PB_7, 115200);
#endif
static X_NUCLEO_53L0A1 *board=NULL;

uint16_t speed_right{};
uint16_t speed_left{};
uint8_t pwm_right{};
uint8_t pwm_left{};

int main() {
  float send[4]{};
  float receive[4]{};
  int status;
  
  DevI2C *device_i2c = new DevI2C(VL53L0_I2C_SDA, VL53L0_I2C_SCL);
    
  /* creates the 53L0A1 expansion board singleton obj */
  // board = X_NUCLEO_53L0A1::instance(device_i2c, A2, D8, D2);
  board = X_NUCLEO_53L0A1::instance(device_i2c);
  /* init the 53L0A1 expansion board with default values */
  status = board->init_board();
  if(status){
    return 1;
  }
  MotorDriver motor_right(D15, D14, D13, D12, A1);
  MotorDriver motor_left(D11, D10, D9, D8, A0);
  RotaryInc rotary(D15,D14,2 * 50.8 * M_PI,200);

  while(1){
    serialSend(send, raspi);
    serialReceive(receive, raspi);
    //get distance data
    /*
    status = board->sensor_centre->get_distance(&distance);
    long long  speed = rotary.getSpeed();
    pc.printf("speed = %lld\n", speed);    
    */
    //output motor
    motor_right.setPwm(pwm_right);
    motor_right.currentLimit();
    motor_left.setPwm(pwm_left);
    motor_left.currentLimit();
  }
}
