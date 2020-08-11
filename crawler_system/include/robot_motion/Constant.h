#ifndef Constant_H_
#define Constant_H_
#include<array>
#include"robot_motion/DXL.h"
using std::array;
namespace FlipperConstant{
  //HACK:一つずつサーボIDを管理するのは頭が悪い
  constexpr int front_right = 0;
  constexpr int front_left = 1;
  constexpr int rear_right = 2;
  constexpr int rear_left = 3;

  constexpr double flipper_m = 100;
  constexpr double flipper_lg = 100;
  constexpr double gravity = 9.81;
};
namespace DXLConstant{
  constexpr double ORIGIN_DEG = 90.0;
  constexpr double AUTO_MAX_DEG = 75.0;
  constexpr double AUTO_MIN_DEG = -90.0;
  constexpr int MAX_POSITION_VALUE = 1048575;
  constexpr int MIN_POSITION_VALUE = -1048575;
  constexpr int DYNAMIXEL_RESOLUTION = 4096;
  constexpr double DYNAMIXEL_RESOLUTION_ANGLE = 0.088;
  constexpr bool TORQUE_ENABLE = 1;
  constexpr bool TORQUE_DISABLE = 0;
  //TODO:それぞれのサーボゲインを設ける
  constexpr double Kp = 1.0;
  constexpr double Kd = 1.0;
};
constexpr std::array<int, 4> dynamixel_num{0, 1, 3, 2};

std::array<double, 4>     ref_DXL_raw_pos{};
std::array<double, 4> current_DXL_raw_pos{};

std::array<double, 4>     ref_DXL_rad{};
std::array<double, 4> current_DXL_rad{};

std::array<double, 4>     ref_DXL_torque{};
std::array<double, 4> current_DXL_torque{};
const DXL::MODE DXL_MODE = DXL::MODE::TORQUE_CONTROL;

#endif