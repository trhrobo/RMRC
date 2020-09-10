/**
 * @file Rotation.cpp

 * @brief Rotationの実装
**/
#include<ros/ros.h>
#include"robot_motion/Rotation.h"
//WARNING:引数が違う気がするint idで本当にいいのか?DXLの配置位置では??

void Rotation::setRotation(const int id, const Rotation::severalType type, const Rotation::setRotationType direction){
    if(DXL_MODE == DXL::MODE::POS_CONTROL){
        int set_id{};
        //NOTE:Type::allではid0の値を他の値にコピーする
        type == severalType::one ? set_id = id : set_id = 0;
        switch(direction){
          case setRotationType::forward:
            ROS_INFO("forward");
            ref_DXL_raw_pos[set_id] = DXLConstant::MAX_POSITION_VALUE; 
            break;
          case setRotationType::reverse:
            ROS_INFO("reverse");
            ref_DXL_raw_pos[set_id] = DXLConstant::MIN_POSITION_VALUE;
            break;
          case setRotationType::nomal:
            ROS_INFO("nomal");
            ref_DXL_raw_pos[set_id] = current_DXL_raw_pos[set_id];
            break;
          default:
              ROS_ERROR("this direction of rotation is invalid");
              break;
        }
        if(type == severalType::all){
            for(int i = 1; i < dynamixel_num.size(); ++i){
                ref_DXL_raw_pos[i] = ref_DXL_raw_pos[0];
            }
        }
    }else if(DXL_MODE == DXL::MODE::TORQUE_CONTROL){
        if(type == severalType::one){
            switch(direction){
                case setRotationType::forward:
                  ref_DXL_torque[i] = 30;
                  break;
                case setRotationType::reverse:
                  ref_DXL_torque[i] = -30;
                  break;
                case setRotationType::nomal:
                  ref_DXL_torque[i] = current_DXL_torque[i];
                  break;
                default:
                  ROS_ERROR("this direction of rotation is invalid");
                  break;
            }
        }else if(type == severalType::all){
            switch(direction){
                case setRotationType::forward:
                  for(auto &ref_torque : ref_DXL_torque){
                    ref_torque = 30; 
                  }
                  break;
                case setRotationType::reverse:
                  for(auto &ref_torque : ref_DXL_torque){
                    ref_torque = -30;
                  }
                  break;
                case setRotationType::nomal:
                  for(int i = 0; i < 4; ++i){
                    ref_DXL_torque[i] = current_DXL_torque[i];
                  }
                  break;
                default:
                    ROS_ERROR("this direction of rotation is invalid");
                    break;
            }
        }
    }
}

void Rotation::reset() {
  for (int i = 0; i < dynamixel_num.size(); ++i) {
    //TODO:template化
    ref_DXL_rad[i] = degToRad<double>(DXLConstant::ORIGIN_DEG);
  }
}