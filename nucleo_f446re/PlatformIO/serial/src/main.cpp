#include "mbed.h"
#include<stdio.h>
#include<cstring>
#include<cstdint>
#define HEAD_BYTE 0xFF
#define STX 0x02
#define float32_t float
//Serial raspi(USBTX, USBRX, 115200);
Serial raspi(PB_6, PB_7, 115200);

uint16_t mg = 0;
uint16_t speed_right = 0;
uint16_t speed_left = 0;
uint8_t dataByte[2];
uint8_t receiveByte[4];
uint8_t checksum_send = 0;
uint8_t checksum_receive = 0;
uint8_t pwm_right = 0;
uint8_t pwm_left = 0;


//send
float32_t send_data[5] = {};
//receive
uint8_t got_data = 0;
uint8_t byte_now = 0;
uint8_t byte[2] = {};
float32_t receive_result[2] = {};

void serialSend(){
  send_data[0] = receive_result[0];
  send_data[1] = receive_result[1];
  send_data[2] = 4.4;
  send_data[3] = 5.5;
  send_data[4] = 6.6;
  uint8_t checksum_send = 0;
  unsigned char data_h_h[5];
  unsigned char data_h_l[5];
  unsigned char data_l_h[5];
  unsigned char data_l_l[5];
  uint32_t byte_divide[5];
  for(int i = 0; i < 5; ++i){
    memcpy(&byte_divide[i], &send_data[i], 4);
    data_h_h[i] = (byte_divide[i] >> 24) & 0xFF;
    data_h_l[i] = (byte_divide[i] >> 16) & 0xFF;
    data_l_h[i] = (byte_divide[i] >> 8) & 0xFF;
    data_l_l[i] = (byte_divide[i] >> 0) & 0xFF;
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
  for(int i = 0; i < 5; ++i){
    for(int k = 0; k < 5; ++k){
      raspi.putc(sendFormat[i][k]);
      checksum_send += sendFormat[i][k];
    }
  }
  //send checksum
  raspi.putc(checksum_send);
}

void serialReceive(){
 uint8_t checksum_receive = 0;
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
  if(got_data == HEAD_BYTE){
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
        int32_t result[2];
        for(int i = 0; i < 2; ++i){
          //receiveFormat[i][0]はidである
          result[i] = static_cast<int32_t>((receiveFormat[i][1] << 24 & 0xFF000000)
                                           | (receiveFormat[i][2] << 16 & 0x00FF0000)
                                           | (receiveFormat[i][3] <<  8 & 0x0000FF00)
                                           | (receiveFormat[i][4] <<  0 & 0x000000FF)
          );
          memcpy(&receive_result[i], &result[i], 4);
        }
      }
    }
  }
}

int main() {
  while(1){
    for(int i = 0; i < 5; ++i){
      send_data[i] = i + 0.5;
    }
    serialSend();
    serialReceive();
  }
}
