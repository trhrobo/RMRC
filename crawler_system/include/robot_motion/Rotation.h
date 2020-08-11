#ifndef Rotation_H_
#define Rotation_H_

#include"robot_motion/DXL.h"
#include"robot_motion/Constant.h"
#include"robot_motion/flipper_util.h"
#include<ros/ros.h>

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
  template<severalType type>
  void setRotation(const int id, const setRotationType direction, DXL::DXLControl<double, DXL::MODE>(&DXLservo)[dynamixel_num.size()]);

  inline void reset();
};

#endif