#pragma once
#include<pigpiod_if2.h>
#include<iostream>
#include<ros.h>

namespace dozap{
class pigpio{
    public:
        //TODO:パラメータパックを使えばいい感じにportを複数設定出来るかも
        pigpio(const int baudrate, const char *port){}
        pigpio& operator=(const pigpio&) = delete;
        pigpio& operator=(pigpio&&) = delete;
        pigpio(const pigpio&) = delete;
        pigpio(pigpio&&) = delete;
        static pigpio& getInstance(){
            return *_instance;   
        }
        static void create(){
            if(!_instance){
                _instance = new pigpio;
                _instance -> _pi = pigpio_start(0, 0);
            }
        }
        static void destroy(){
            delete _instance;
            _instance = nullptr;
        }
        void uart_open(const int baudrate, const char *port){
            try {
                unsigned char dummy_flag{};
                _serial_handle = serial_open(_pi, const_cast<char *>(port), baudrate, dummy_flag);
                if (_serial_handle < 0) {
                    throw serial_handle;
                } else {
                    ROS_INFO("Serial Initialize complete");
                }
            }
            catch (int _serial_handle) {
                ROS_INFO("Serial Initialize Failed");
                return 1;
            }
        }
        void uart_write(uint8_t write_data){
            serial_write_byte(_pi, _serial_handle, write_data);
        }
        uint8_t uart_read(){
            return  static_cast<uint8_t>(serial_read_byte(_pi, _serial_handle));
        }
        void i2c_open(int32_t dev_id){
            gpio_handle_ = Pigpiod::gpio().checkHandle();
            uint8_t dummy_flag = 0;
            i2c_handle_ = i2c_open(gpio_handle_, 1, dev_id, dummy_flag);
            return i2c_handle_ < 0 ? 0 : 1;
        }
        void i2c_write(int32_t reg, int32_t data){
            i2c_write_byte_data(gpio_handle_, i2c_handle_, reg, data);
        }
        uint8_t i2c_read(int32_t reg){
            return static_cast<uint8_t>(i2c_read_byte_data(gpio_handle_, i2c_handle_, reg));
        }
    private:
        pigpio() = default;
        ~pigpio() = default;
        static pigpio *_instance;
        const int _baudrate;
        const char *_port;
        int32_t _pi;
        int32_t _serial_handle;
}; 
}