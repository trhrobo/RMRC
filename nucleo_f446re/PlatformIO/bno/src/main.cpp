#include"mbed.h"
#include"BNO055.h"
 
Serial pc(USBTX, USBRX);
BNO055 imu(D14, D15);
 
int main() {
    pc.baud(115200);
    pc.printf("BNO055 Hello World\n");
// Reset the BNO055
    imu.reset();
    while (1) {
        /*
        if(!imu.check()){
            imu.reset();
            pc.printf("failed\n");
        }*/
        imu.setmode(OPERATION_MODE_IMUPLUS);
        imu.get_angles();
        pc.printf("%f\n",imu.euler.yaw);
    }
}
 