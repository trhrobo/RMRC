/**
 * @file DXL.h

 * @brief DXLの宣言
**/
#pragma once
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
      bool operator()();
    private:
      const int DXL_ID;
      bool (DXLControl::*funcp)(T);
  };
  template<typename T>
  int dynamixelSet(T goal_angle, T now_pos);
};