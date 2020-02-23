#include"mbed.h"
//#include"VNH5019.h"
#include"AMT102.h"
#include"x_nucleo_53l0a1.h"
#include"VNH5019.h"
#include<stdio.h>
#include"rtos.h"
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
Thread sendThread;
Thread receiveThread;

uint16_t mg = 0;
uint16_t speed_right{};
uint16_t speed_left{};
uint8_t dataByte[2];
uint8_t receiveByte[4];
uint8_t checksum_send{};
uint8_t checksum_receive{};
uint8_t pwm_right{};
uint8_t pwm_left{};

void serialSend(){
    //serial send magnet data;
    dataByte[0] = (mg >> 8) & 0xFF;
    dataByte[1] = (mg >> 0) & 0xFF;
    // send.putc(HEAD_BYTE);
    raspi.putc(0x02);

    for (int i = 0; i < 2; ++i) {
      if ((dataByte[i] == 0x7D) || (dataByte[i] == 0x02)) {
        raspi.putc(0x7D);
        checksum_send += 0x7D;
        raspi.putc(dataByte[i] ^ 0x20);
        checksum_send += dataByte[i] ^ 0x20;
      } else {
        raspi.putc(dataByte[i]);
        checksum_send += dataByte[i];
      }
    }
    raspi.putc(checksum_send);
}

void serialReceive(){

    if (static_cast<uint8_t>(raspi.getc()) == 0x02) {
      for (int i = 0; i < 4; ++i) {
        uint8_t nowByte = static_cast<uint8_t>(raspi.getc());
        if (nowByte == 0x7D) {
          checksum_receive += 0x7D;
          uint8_t nextByte = static_cast<uint8_t>(raspi.getc());
          receiveByte[i] = nextByte ^ 0x20;
          checksum_receive += nextByte;
        } else {
          receiveByte[i] = nowByte;
          checksum_receive += nowByte;
        }
      }
      uint8_t checksum = static_cast<uint8_t>(raspi.getc());
      if (checksum_receive == checksum) {
        speed_right = static_cast<uint16_t>(((receiveByte[0] << 8) && 0xFF00) |
                                            ((receiveByte[1] << 0) && 0x00FF));
        speed_left = static_cast<uint16_t>(((receiveByte[2] << 8) && 0xFF00) |
                                           ((receiveByte[3] << 0) && 0x00FF));
      }
    }
}

int main() {
  sendThread.start(serialSend);
  receiveThread.start(serialReceive);
  int status;
  uint32_t distance;
  DevI2C *device_i2c = new DevI2C(VL53L0_I2C_SDA, VL53L0_I2C_SCL);

  /* creates the 53L0A1 expansion board singleton obj */
  // board = X_NUCLEO_53L0A1::instance(device_i2c, A2, D8, D2);
  board = X_NUCLEO_53L0A1::instance(device_i2c);
  /* init the 53L0A1 expansion board with default values */
  status = board->init_board();
  if(status){
    return 1;
  }
  #if DEBUG
  RotaryInc rotary(D15,D14,2 * 50.8 * M_PI,200);
  #else
/*
  MotorDriverPin pin_right;
  pin_right.pin_A = 1;
  pin_right.pin_B = 2;
  pin_right.pin_CS = 3;
  pin_right.pin_EN = 4;
  pin_right.pin_PWM = 5;

  MotorDriverPin pin_left;
  pin_left.pin_A = 6;
  pin_left.pin_B = 7;
  pin_left.pin_CS = 8;
  pin_left.pin_EN = 9;
  pin_left.pin_PWM = 10;
*/
  MotorDriver motor_right(D15, D14, D13, D12, A1);
  MotorDriver motor_left(D11, D10, D9, D8, A0);
  RotaryInc rotary(D15,D14,2 * 50.8 * M_PI,200);
  #endif

  while (1) {
    //get distance data
    status = board->sensor_centre->get_distance(&distance);
    #if DEBUG
    long long  speed = rotary.getSpeed();
    pc.printf("speed = %lld\n", speed);
    //get magnet data
    if (magnet.read()) {
      mg = 1;
    } else {
      mg = 0;
    }
    #else
    //output motor
    motor_right.setPwm(pwm_right);
    motor_right.currentLimit();
    motor_left.setPwm(pwm_left);
    motor_left.currentLimit();
    #endif
  }
  sendThread.join();
  receiveThread.join();
}

