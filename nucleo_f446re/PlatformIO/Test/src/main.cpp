#include <mbed.h>

Serial send(PB_6, PB_7, 115200);
uint16_t ppm = 0;
int main() {
uint8_t dataByte[2];
uint8_t check_sum{};
  while(1){
    if(ppm > 500){ppm = 0;}
    ++ppm;
    dataByte[0] = (ppm >> 8) & 0xFF;
    dataByte[1] = (ppm >> 0) & 0xFF;
    //send.putc(HEAD_BYTE);
    send.putc(0x02);
  
    for(int i = 0; i < 2; ++i){
      if((dataByte[i] == 0x7D) || (dataByte[i] == 0x02)){
        send.putc(0x7D);
        check_sum += 0x7D;
        send.putc(dataByte[i] ^ 0x20);
        check_sum += dataByte[i] ^ 0x20;
      }else{
        send.putc(dataByte[i]);
        check_sum += dataByte[i];
      }
    }
    send.putc(check_sum);
  }
}