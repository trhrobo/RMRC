#include <iostream>
#include <pigpiod_if2.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;
const char *port = "/dev/ttyAMA0";
int baudrate = 115200;

int main(int argc, char **argv) {
  ros::init(argc, argv, "stm_serial");
  ros::NodeHandle n;
  ros::Publisher sensor_pub =
    n.advertise<std_msgs::Float64MultiArray>("/nucleo/sensor_info", 30);
  ros::Rate loop_rate(100);

  int pi = pigpio_start(0, 0);
  int serial_handle{};
  unsigned char dummy_flag{};

  try {
    serial_handle =
      serial_open(pi, const_cast<char *>(port), baudrate, dummy_flag);
    if (serial_handle < 0) {
      throw serial_handle;
    } else {
      cout << "Serial Initialize complete" << endl;
    }
  }

  catch (int _serial_handle) {
    cout << "Serial Initialize Failed" << endl;
    return 1;
  }

  uint8_t got_data{};
  uint8_t byte_now{};
  uint8_t byte[2]{};
  uint16_t result{};
  std_msgs::Float64MultiArray msg_receive;
  msg_receive.data.resize(1);
  while (ros::ok()) {
    uint8_t checksum{};
    got_data = static_cast<uint8_t>(serial_read_byte(pi, serial_handle));
    if (got_data == 0x02) {
      for (int i = 0; i < 2; ++i) {
        byte_now = static_cast<uint8_t>(serial_read_byte(pi, serial_handle));
        if (byte_now == 0x7D) {
          checksum += 0x7D;
          uint8_t next_byte =
            static_cast<uint8_t>(serial_read_byte(pi, serial_handle));
          byte[i] = next_byte ^ 0x20;
          checksum += next_byte;
        } else {
          byte[i] = byte_now;
          checksum += byte_now;
        }
      }
      uint8_t checksum_receive =
        static_cast<uint8_t>(serial_read_byte(pi, serial_handle));
      if (checksum_receive == checksum) {
        result = static_cast<uint16_t>(((byte[0] << 8) & 0xFF00) |
            ((byte[1] << 0) & 0x00FF));
        cout << "result : " << result << endl;
      }
    }
    msg_receive.data[0] = result;
    sensor_pub.publish(msg_receive);
    ros::spinOnce();
    loop_rate.sleep();
  }
  serial_close(pi, serial_handle);
}
