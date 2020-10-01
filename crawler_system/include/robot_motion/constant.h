/**
 * @file Constant.h

 * @brief Constantの宣言
**/
#ifndef CONSTANT
#define CONSTANT

#include<array>
namespace dxl{
  enum class Mode{
    pos_control,
    torque_control
  };
};
namespace flipper_constant{
  //HACK:一つずつサーボIDを管理するのは頭が悪い
  inline constexpr int front_right = 0;
  inline constexpr int front_left = 1;
  inline constexpr int rear_right = 2;
  inline constexpr int rear_left = 3;

  inline constexpr double flipper_m = 100;
  inline constexpr double flipper_lg = 100;
  inline constexpr double gravity = 9.81;
};
namespace dxl_constant{
  inline constexpr double origin_deg = 90.0;
  inline constexpr double auto_max_deg = 75.0;
  inline constexpr double auto_min_deg = -90.0;
  inline constexpr int max_position_value = 1048575;
  inline constexpr int min_position_value = -1048575;
  inline constexpr int dynamixel_resolution = 4096;
  inline constexpr double dynamixel_resolution_angle = 0.088;
  inline constexpr bool torque_enable = 1;
  inline constexpr bool torque_disable = 0;
  //TODO:それぞれのサーボゲインを設ける
  inline constexpr double Kp = 1.0;
  inline constexpr double Kd = 1.0;
};

inline std::array<double, 4>     ref_dxl_raw_pos{};
inline std::array<double, 4> current_dxl_raw_pos{};

//std::array<double, 4>     ref_DXL_rad{};
inline double ref_dxl_rad[4]{};
inline std::array<double, 4> current_dxl_rad{};
inline std::array<double, 4> current_dxl_rad_raw{};

inline std::array<double, 4>     ref_dxl_torque{};
inline std::array<double, 4> current_dxl_torque{};
constexpr dxl::Mode dxl_mode = dxl::Mode::pos_control;
constexpr bool flag_dist = true;
inline std::array<int, 4> dynamixel_num{0, 1, 3, 2};

#endif