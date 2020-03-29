#ifndef SERIAL
#define SERIAL
#include "mbed.h"
#include<stdio.h>
#include<cstring>
#include<cstdint>
#include<array>
#define HEAD_BYTE 0xFF
#define STX 0x02
#define float32_t float

class SerialRaspi{
    public:
        SerialRaspi(PinName TX1, PinName RX2);
        void serialMain(std::array<float32_t, 5> &data_send, std::array<float32_t, 2> &data_receive);
        void serialSend(std::array<float32_t, 5> &send_data);
        void serialReceive();
    private:
        Serial *raspi;
        float32_t receive_result[2] = {};
};
#endif