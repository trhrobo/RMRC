#ifndef DXL_H_
#define DXL_H_

#include"robot_motion/Rotation.h"
#include"robot_motion/Constant.h"
#include"robot_motion/flipper_util.h"
#include<ros/ros.h>
namespace DXL{
  enum class MODE{
    POS_CONTROL,
    TORQUE_CONTROL
  };
  template<typename T, MODE dxl_mode>
  class DXLControl{
    public:
      DXLControl(int _ID);
      bool TorqueControl(T theta_d);
      bool PosControl(T theta_d);
      bool PosDirect();
      bool operator()(T theta_d);
    private:
      const int DXL_ID;
      int (*funcp)(int, int);
  };
  template<typename T>
  int dynamixelSet(T goal_angle, T now_pos);
};
#endif