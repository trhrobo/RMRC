//#include <BufferedSerial.h>
#include <mbed.h>

constexpr double threshold = 1.0;
DigitalIn mgnet(D13);

struct pin {
  int pin_A;
  int pin_B;
  int pin_PWM;
  int pin_CS;
  int pin_EN;
  // pin(int user_A, int user_B, int user_PWM, int user_US, int
  // user_EN):pin_A(user_A), pin_B(user_B), pin_PWM(user_PWM), pin_CS(user_US),
  // pin_EN(user_EN){}
};

class VNH5019 {
private:
  pin object;

public:
  VNH5019(const pin user);
  void set(int pwm);
  void current_limit();
};

VNH5019::VNH5019(const pin user) { object = user; }

void VNH5019::set(int pwm) {
  if (pwm > 0) {
    this->object.pin_A = 1;
    this->object.pin_B = 0;
    this->object.pin_PWM = (float)pwm / 255;
  } else {
    this->object.pin_A = 0;
    this->object.pin_B = 1;
    this->object.pin_PWM = (float)pwm / -255;
  }
}

void VNH5019::current_limit() {
  if (this->object.pin_CS > threshold) {
    this->object.pin_EN = 0;
  } else {
    this->object.pin_EN = 1;
  }
}

DigitalOut VNHpin_A(D1);
DigitalOut VNHpin_B(D2);
DigitalIn VNHpin_enable(D3);
AnalogOut VNHpin_PWM(D4);
AnalogIn VNHpin_CS(D5);

// BufferedSerial raspi(PB_6, PB_7, 115200);
Serial raspi(PB_6, PB_7, 115200);
int main() {
  uint8_t dataByte[2];
  uint8_t receiveByte[4];
  uint8_t checksum_send{};
  uint8_t checksum_receive{};
  uint16_t mg = 0;
  uint8_t pwm_right{};
  uint8_t pwm_left{};
  uint16_t speed_right{};
  uint16_t speed_left{};

  pin pin_right;
  pin_right.pin_A = 1;
  pin_right.pin_B = 2;
  pin_right.pin_CS = 3;
  pin_right.pin_EN = 4;
  pin_right.pin_PWM = 5;

  pin pin_left;
  pin_left.pin_A = 6;
  pin_left.pin_B = 7;
  pin_left.pin_CS = 8;
  pin_left.pin_EN = 9;
  pin_left.pin_PWM = 10;
  VNH5019 motor_right(pin_right);
  VNH5019 motor_left(pin_left);

  while (1) {
    if (mgnet.read()) {
      mg = 1;
    } else {
      mg = 0;
    }
    dataByte[0] = (mg >> 8) & 0xFF;
    dataByte[1] = (mg >> 0) & 0xFF;
    // send.putc(HEAD_BYTE);
    raspi.putc(0x02);

    for (int i = 0; i < 2; ++i) {
      if ((dataByte[i] == 0x7D) || (dataByte[i] == 0x02)) {
        raspi.putc(0x7D);
        checksum_send += 0x7D;
        raspi.putc(dataByte[i] ^ 0x20);
        checksum_send += dataByte[i] ^ 0x20;
      } else {
        raspi.putc(dataByte[i]);
        checksum_send += dataByte[i];
      }
    }
    raspi.putc(checksum_send);

    if (static_cast<uint8_t>(raspi.getc()) == 0x02) {
      for (int i = 0; i < 4; ++i) {
        uint8_t nowByte = static_cast<uint8_t>(raspi.getc());
        if (nowByte == 0x7D) {
          checksum_receive += 0x7D;
          uint8_t nextByte = static_cast<uint8_t>(raspi.getc());
          receiveByte[i] = nextByte ^ 0x20;
          checksum_receive += nextByte;
        } else {
          receiveByte[i] = nowByte;
          checksum_receive += nowByte;
        }
      }
      uint8_t checksum = static_cast<uint8_t>(raspi.getc());
      if (checksum_receive == checksum) {
        speed_right = static_cast<uint16_t>(((receiveByte[0] << 8) && 0xFF00) |
                                            ((receiveByte[1] << 0) && 0x00FF));
        speed_left = static_cast<uint16_t>(((receiveByte[2] << 8) && 0xFF00) |
                                           ((receiveByte[3] << 0) && 0x00FF));
      }
    }
    motor_right.set(pwm_right);
    motor_right.current_limit();
    motor_left.set(pwm_left);
    motor_left.current_limit();
  }
}
