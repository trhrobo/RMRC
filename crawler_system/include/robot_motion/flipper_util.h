/**
 * @file flipper_util.h

 * @brief flipper_utilの宣言
**/
#pragma once
#include<cmath>

template<typename T>
inline T degToRad(const T deg){
  return deg * (M_PI / 180.0);
}
template<typename T>
inline T radToDeg(const T rad){
  return rad * (180.0 / M_PI);
}