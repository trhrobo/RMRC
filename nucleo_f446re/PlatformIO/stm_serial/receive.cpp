#include<iostream>
#include<pigpiod_if2.h>

using namespace std;
const char *port = "/dev/ttyAMA0";
int baudrate = 115200;
unsigned char dummy_flag = 0;

int main(){
	int pi = pigpio_start(0, 0);
        unsigned char dummy_flag = 0;
        int serial_handle = serial_open(pi, const_cast<char *>(port), baudrate, dummy_flag);
	if(serial_handle < 0){
		cout << "Serial Initialize Failed" << endl;
	}else{
		cout << "Serial Initialize Complete" << endl;
	}
	//set_mode(pi, 21, PI_OUTPUT);
	int8_t got_data;
	while(1){
		//gpio_write(pi, 21, 1);
		got_data = static_cast<int8_t>(serial_read_byte(pi, serial_handle));
		cout << got_data << endl;
	}
	serial_close(pi, serial_handle);
}
