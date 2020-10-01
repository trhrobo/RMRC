/**
 * @file Rotation.h

 * @brief Rotationの宣言
**/
#pragma once
#include<array>
#include"robot_motion/flipper_util.h"
#include"robot_motion/constant.h"
namespace rotation{
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
  void setRotation(const int id, const severalType type, const setRotationType direction);

  void reset();
};