/**
 * @file Rotation.h

 * @brief Rotationの宣言
**/
#pragma once
#include<array>
#include"robot_motion/Constant.h"
namespace DXL{
  enum class MODE;
  
  template<typename T, MODE dxl_mode>
  class DXLControl;
};
namespace Rotation{
  enum class setRotationType{
    forward,
    reverse,
    nomal
  };
  enum class severalType{
    one,
    all
  };
  //WARNING:引数が違う気がするint idで本当にいいのか?DXLの配置位置では??
  void setRotation(const int id, const severalType type, const setRotationType direction, DXL::DXLControl<double, DXL_MODE>(&DXLservo)[dynamixel_num.size()]);

  void reset();
};
