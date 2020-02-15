#include "mbed.h"

Serial send(PB_6, PB_7, 115200);

int main(){
  //receive
  uint8_t dataByte[2];
  uint8_t checksum_send;
  uint16_t mg = 0;
  //send
  uint8_t got_data = 0;
  uint8_t byte_now = 0;
  uint8_t byte[2] = {};
  uint16_t result = 0;
  while(1){
    //receive
    uint8_t checksum = 0;
    got_data = send.getc();
    if(got_data == 0x02){
      for(int i = 0; i < 2; ++i){
        byte_now = send.getc();
        if(byte_now == 0x7D){
          checksum += 0x7D;
          uint8_t next_byte = send.getc();
          byte[i] = next_byte ^ 0x20;
          checksum += next_byte;
        }else{
          byte[i] = byte_now;
          checksum += byte_now;
        }
      }
      uint8_t checksum_receive = send.getc();
      if(checksum_receive == checksum){
        result = static_cast<uint16_t>(((byte[0] << 8) & 0xFF00) | ((byte[1] << 0) & 0x00FF));
      }
    }
    //send
    checksum_send = 0;
    mg = result;
    dataByte[0] = (mg >> 8) & 0xFF;
    dataByte[1] = (mg >> 0) & 0xFF;
    // send.putc(HEAD_BYTE);
    send.putc(0x02);
    while(!send.writeable());
    for (int i = 0; i < 2; ++i) {
      if ((dataByte[i] == 0x7D) || (dataByte[i] == 0x02)) {
        send.putc(0x7D);
        while(!send.writeable());
        checksum_send += 0x7D;
        send.putc(dataByte[i] ^ 0x20);
        while(!send.writeable());
        checksum_send += dataByte[i] ^ 0x20;
      } else {
        send.putc(dataByte[i]);
        while(!send.writeable());
        checksum_send += dataByte[i];
      }
    }
    send.putc(checksum_send);
  }
}
