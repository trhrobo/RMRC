#include <mbed.h>

DigitalOut led(LED2);

int main() {

  // put your setup code here, to run once:

  while(1) {
    led = 1;
    wait(0.2);
    led = 0;
    wait(0.2);
    // put your main code here, to run repeatedly:
  }
}