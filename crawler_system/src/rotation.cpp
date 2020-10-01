/**
 * @file Rotation.cpp

 * @brief Rotationの実装
**/
#include<ros/ros.h>
#include"robot_motion/rotation.h"
//WARNING:引数が違う気がするint idで本当にいいのか?DXLの配置位置では??

void rotation::setRotation(const int id, const rotation::severalType type, const rotation::setRotationType direction){
    if(dxl_mode == dxl::Mode::pos_control){
        int set_id{};
        //NOTE:Type::allではid0の値を他の値にコピーする
        type == severalType::one ? set_id = id : set_id = 0;
        switch(direction){
          case setRotationType::forward:
            ROS_INFO("forward");
            ref_dxl_raw_pos[set_id] = dxl_constant::max_position_value; 
            break;
          case setRotationType::reverse:
            ROS_INFO("reverse");
            ref_dxl_raw_pos[set_id] = dxl_constant::min_position_value;
            break;
          case setRotationType::nomal:
            ROS_INFO("nomal");
            ref_dxl_raw_pos[set_id] = current_dxl_raw_pos[set_id];
            break;
          default:
              ROS_ERROR("this direction of rotation is invalid");
              break;
        }
        if(type == severalType::all){
            for(int i = 1; i < dynamixel_num.size(); ++i){
                ref_dxl_raw_pos[i] = ref_dxl_raw_pos[0];
            }
        }
    }else if(dxl_mode == dxl::Mode::torque_control){
        if(type == severalType::one){
            switch(direction){
                case setRotationType::forward:
                  ref_dxl_torque[id] = 30;
                  break;
                case setRotationType::reverse:
                  ref_dxl_torque[id] = -30;
                  break;
                case setRotationType::nomal:
                  ref_dxl_torque[id] = current_dxl_torque[id];
                  break;
                default:
                  ROS_ERROR("this direction of rotation is invalid");
                  break;
            }
        }else if(type == severalType::all){
            switch(direction){
                case setRotationType::forward:
                  for(auto &ref_torque : ref_dxl_torque){
                    ref_torque = 30; 
                  }
                  break;
                case setRotationType::reverse:
                  for(auto &ref_torque : ref_dxl_torque){
                    ref_torque = -30;
                  }
                  break;
                case setRotationType::nomal:
                  for(int i = 0; i < 4; ++i){
                    ref_dxl_torque[id] = current_dxl_torque[i];
                  }
                  break;
                default:
                    ROS_ERROR("this direction of rotation is invalid");
                    break;
            }
        }
    }
}

void rotation::reset() {
  for (int i = 0; i < dynamixel_num.size(); ++i) {
    //TODO:template化
    ref_dxl_rad[i] = degToRad<double>(dxl_constant::origin_deg);
  }
}