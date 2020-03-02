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
#define HEAD_BYTE 0xFF
#define STX 0x02
#define float32_t float
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


//send
uint16_t send_data[5]{};
//receive
uint8_t got_data{};
uint8_t byte_now{};
uint8_t byte[2]{};
uint32_t receive_result[2]{};

void serialSend(){ 
  uint8_t checksum_send = 0;
  unsigned char data_h_h[5];
  unsigned char data_h_l[5];
  unsigned char data_l_h[5];
  unsigned char data_l_l[5];
  for(int i = 0; i < 5; ++i){
    data_h_h[i] = (send_data[i] >> 24) & 0xFF;
    data_h_l[i]= (send_data[i] >> 16) & 0xFF;
    data_l_h[i] = (send_data[i] >> 8) & 0xFF;
    data_l_l[i] = (send_data[i] >> 0) & 0xFF;
  }
  unsigned char sendFormat[5][5] = {
    {0, data_h_h[0], data_h_l[0], data_l_h[0], data_l_l[0]},
    {1, data_h_h[1], data_h_l[1], data_l_h[1], data_l_l[1]},
    {2, data_h_h[2], data_h_l[2], data_l_h[2], data_l_l[2]},
    {3, data_h_h[3], data_h_l[3], data_l_h[3], data_l_l[3]},
    {4, data_h_h[4], data_h_l[4], data_l_h[4], data_l_l[4]},
  };
  //send head byte
  raspi.putc(HEAD_BYTE);
  checksum_send += HEAD_BYTE;
  raspi.putc(STX);
  checksum_send += STX;
  for(int i = 0; i < 2; ++i){
    for(int k = 0; k < 5; ++k){
      raspi.putc(sendFormat[i][k]);
      checksum_send += sendFormat[i][k];
    }
  }
  //send checksum
  raspi.putc(checksum_send);
}

void serialReceive(){
 uint8_t checksum_receive{};
  uint8_t receive_data[5];
  unsigned char data_h_h[2];
  unsigned char data_h_l[2];
  unsigned char data_l_h[2];
  unsigned char data_l_l[2];
  unsigned char receiveFormat[2][5] = {
    {0, data_h_h[0], data_h_l[0], data_l_h[0], data_l_l[0]},
    {1, data_h_h[1], data_h_l[1], data_l_h[1], data_l_l[1]},
  };
  got_data = static_cast<uint8_t>(raspi.getc());
  if(got_data = HEAD_BYTE){
    got_data = static_cast<uint8_t>(raspi.getc());
    if(got_data == STX){
      checksum_receive += HEAD_BYTE;
      checksum_receive += STX;
      for(int k = 0; k < 2; ++k){
        for(int i = 0; i < 5; ++i){
          receive_data[i] = static_cast<uint8_t>(raspi.getc());
          checksum_receive += receive_data[i];
          receiveFormat[receive_data[0]][i] = receive_data[i];
        }
      }
      got_data = static_cast<uint8_t>(raspi.getc());
      if(got_data == checksum_receive){
        for(int i = 0; i < 2; ++i){
          //receiveFormat[i][0]はidである
          receive_result[i] = static_cast<float32_t>((receiveFormat[i][1] << 24 & 0xFF000000)
                                           | (receiveFormat[i][2] << 16 & 0x00FF0000)
                                           | (receiveFormat[i][3] <<  8 & 0x0000FF00)
                                           | (receiveFormat[i][4] <<  0 & 0x000000FF)
          );
        }
      }
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
