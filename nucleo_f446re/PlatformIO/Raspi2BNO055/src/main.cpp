#include"mbed.h"
#include"BNO055.h"

Serial send(USBTX, USBRX, 115200);

int main(){
    uint8_t dataByte[2];
    uint8_t checksum_send;
    uint16_t mg = 0;
    BNO055 imu(D14, D15);
    while(1){
        //gyro
        imu.get_angles();
        //mg = imu.euler.yaw;
        int8_t data = imu.euler.yaw;
        //send
        //checksum_send = 0;
        mg = 1;
        if(imu.check() == false){
            send.printf("succes ");
            send.printf("%d\n", data);
        }else{
            send.printf("failed\n");
        }
        /*
        mg++;
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
        */
    }
}