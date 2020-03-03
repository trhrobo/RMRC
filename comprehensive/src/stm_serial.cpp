#include<iostream>
#include<pigpiod_if2.h>
#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>
#include<thread>
#include<cstdint>
#include<cstring>

using std::thread;
using std::cout;
using std::endl;
using std::memcpy;

#define HEAD_BYTE 0xFF
#define STX 0x02
#define float32_t float

const char *port = "/dev/ttyAMA0";
int baudrate = 115200;
int pi = pigpio_start(0, 0);
int serial_handle{};
//send
float32_t send_data[2]{};
//receive
uint8_t got_data{};
uint8_t byte_now{};
uint8_t byte[2]{};
float32_t receive_result[5]{};

typedef struct{
  uint16_t gyro_data;
  uint16_t tof_data[4];
}ReceiveFormat;

void sendSerial(){
  uint8_t checksum_send = 0;
  unsigned char data_h_h[2], data_h_l[2], data_l_h[2], data_l_l[2];
  //divide byte
  int32_t byte_divide[2];
  for(int i = 0; i < 2; ++i){
    //float32_t to int32_t
    memcpy(&byte_divide[i], &send_data[i], 4);
    data_h_h[i] = (byte_divide[i] >> 24) & 0xFF;
    data_h_l[i] = (byte_divide[i] >> 16) & 0xFF;
    data_l_h[i] = (byte_divide[i] >> 8) & 0xFF;
    data_l_l[i] = (byte_divide[i] >> 0) & 0xFF;
  };
  unsigned char sendFormat[2][5] = {
    {0, data_h_h[0], data_h_l[0], data_l_h[0], data_l_l[0]},
    {1, data_h_h[1], data_h_l[1], data_l_h[1], data_l_l[1]},
  };
  //send head byte
  serial_write_byte(pi, serial_handle, HEAD_BYTE);
  checksum_send += HEAD_BYTE;
  serial_write_byte(pi, serial_handle, STX);
  checksum_send += STX;
  for(int i = 0; i < 2; ++i){
    for(int k = 0; k < 5; ++k){
      serial_write_byte(pi, serial_handle, sendFormat[i][k]);
      checksum_send += sendFormat[i][k];
    }
  }
  //send checksum
  serial_write_byte(pi, serial_handle, checksum_send);
}
void receiveSerial(){
  uint8_t checksum_receive{};
  uint8_t receive_data[5];
  unsigned char data_h_h[5], data_h_l[5], data_l_h[5], data_l_l[5];
  unsigned char receiveFormat[5][5] = {
    {0, data_h_h[0], data_h_l[0], data_l_h[0], data_l_l[0]},
    {1, data_h_h[1], data_h_l[1], data_l_h[1], data_l_l[1]},
    {2, data_h_h[2], data_h_l[2], data_l_h[2], data_l_l[2]},
    {3, data_h_h[3], data_h_l[3], data_l_h[3], data_l_l[3]},
    {4, data_h_h[4], data_h_l[4], data_l_h[4], data_l_l[4]},
  };
  got_data = static_cast<uint8_t>(serial_read_byte(pi, serial_handle));
  if(got_data == HEAD_BYTE){
    got_data = static_cast<uint8_t>(serial_read_byte(pi, serial_handle));
    if(got_data == STX){
      checksum_receive += HEAD_BYTE;
      checksum_receive += STX;
      for(int k = 0; k < 5; ++k){
        for(int i = 0; i < 5; ++i){
          receive_data[i] = static_cast<uint8_t>(serial_read_byte(pi, serial_handle));
          checksum_receive += receive_data[i];
          receiveFormat[receive_data[0]][i] = receive_data[i];
        }
      }
      got_data = static_cast<uint8_t>(serial_read_byte(pi, serial_handle));
      if(got_data == checksum_receive){
        int32_t result[5];
        for(int i = 0; i < 5; ++i){
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
void sendCallback(const std_msgs::Float32MultiArray &msg){
  send_data[0] = msg.data[0];
  send_data[1] = msg.data[1];
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "stm_serial");
  ros::NodeHandle n;
  ros::Publisher sensor_pub = n.advertise<std_msgs::Float32MultiArray>("/nucleo/sensor_info", 30);
  ros::Subscriber serial_send_sub = n.subscribe("motor_speed", 10, sendCallback);
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

  std_msgs::Float32MultiArray msg_receive;
  msg_receive.data.resize(5);

  thread send(sendSerial);
  thread receive(receiveSerial);

  while (ros::ok()) {
    thread send(sendSerial);
    thread receive(receiveSerial);
    send.join();
    receive.join();
    for(int i = 0; i < 5; ++i){
      msg_receive.data[i] = receive_result[i];
    }
    sensor_pub.publish(msg_receive);
    ros::spinOnce();
    loop_rate.sleep();
  }
  serial_close(pi, serial_handle);
}
