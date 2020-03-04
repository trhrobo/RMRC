#include "mbed.h"
#include "BNO055.h"
 
Serial pc(USBTX, USBRX);
BNO055 imu(PB_9,PB_8);
DigitalOut led(LED1);
 
int main() {
    pc.baud(115200);
    pc.printf("BNO055 Hello World\r\n\r\n");
    led = 1;
// Reset the BNO055
    imu.reset();
// Check that the BNO055 is connected and flash LED if not
    if (!imu.check())
        while (true){
            led = !led;
            wait(0.1);
            }
// Display sensor information
    pc.printf("BNO055 found\r\n\r\n");
    pc.printf("Chip          ID: %0z\r\n",imu.ID.id);
    pc.printf("Accelerometer ID: %0z\r\n",imu.ID.accel);
    pc.printf("Gyroscope     ID: %0z\r\n",imu.ID.gyro);
    pc.printf("Magnetometer  ID: %0z\r\n\r\n",imu.ID.mag);
    pc.printf("Firmware version v%d.%0d\r\n",imu.ID.sw[0],imu.ID.sw[1]);
    pc.printf("Bootloader version v%d\r\n\r\n",imu.ID.bootload);
// Display chip serial number
    for (int i = 0; i<4; i++){
        pc.printf("%0z.%0z.%0z.%0z\r\n",imu.ID.serial[i*4],imu.ID.serial[i*4+1],imu.ID.serial[i*4+2],imu.ID.serial[i*4+3]);
    }
    pc.printf("\r\n");
    while (true) {
        imu.setmode(OPERATION_MODE_NDOF);
        imu.get_calib();
        imu.get_angles();
        pc.printf("%0z %5.1d %5.1d %5.1d\r\n",imu.calib,imu.euler.roll,imu.euler.pitch,imu.euler.yaw);
        wait(1.0);
    }
}
 