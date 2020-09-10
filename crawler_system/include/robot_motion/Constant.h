/**
 * @file Constant.h

 * @brief Constantの宣言
**/
#ifndef CONSTANT
#define CONSTANT

#include<array>
namespace DXL{
  enum class MODE{
    POS_CONTROL,
    TORQUE_CONTROL
  };
};
namespace FlipperConstant{
  //HACK:一つずつサーボIDを管理するのは頭が悪い
  inline constexpr int front_right = 0;
  inline constexpr int front_left = 1;
  inline constexpr int rear_right = 2;
  inline constexpr int rear_left = 3;

  inline constexpr double flipper_m = 100;
  inline constexpr double flipper_lg = 100;
  inline constexpr double gravity = 9.81;
};
namespace DXLConstant{
  inline constexpr double ORIGIN_DEG = 90.0;
  inline constexpr double AUTO_MAX_DEG = 75.0;
  inline constexpr double AUTO_MIN_DEG = -90.0;
  inline constexpr int MAX_POSITION_VALUE = 1048575;
  inline constexpr int MIN_POSITION_VALUE = -1048575;
  inline constexpr int DYNAMIXEL_RESOLUTION = 4096;
  inline constexpr double DYNAMIXEL_RESOLUTION_ANGLE = 0.088;
  inline constexpr bool TORQUE_ENABLE = 1;
  inline constexpr bool TORQUE_DISABLE = 0;
  //TODO:それぞれのサーボゲインを設ける
  inline constexpr double Kp = 1.0;
  inline constexpr double Kd = 1.0;
};

inline std::array<double, 4>     ref_DXL_raw_pos{};
inline std::array<double, 4> current_DXL_raw_pos{};

//std::array<double, 4>     ref_DXL_rad{};
inline double ref_DXL_rad[4]{};
inline std::array<double, 4> current_DXL_rad{};
inline std::array<double, 4> current_DXL_rad_raw{};

inline std::array<double, 4>     ref_DXL_torque{};
inline std::array<double, 4> current_DXL_torque{};
constexpr DXL::MODE DXL_MODE = DXL::MODE::POS_CONTROL;
const std::array<int, 4> dynamixel_num{0, 1, 3, 2};

#endif