#include"robot_motion/Rotation.h"

//WARNING:引数が違う気がするint idで本当にいいのか?DXLの配置位置では??
template<severalType type>
void Rotation::setRotation(const int id, const setRotationType direction, DXL::DXLControl<double, DXL::MODE>(&DXLservo)[dynamixel_num.size()]){
    if(DXL_MODE == DXL::MODE::POS_CONTROL){
        int set_id{};
        //NOTE:Type::allではid0の値を他の値にコピーする
        type == severalType::one ? set_id = id : set_id = 0;
        switch(direction){
          case setRotationType::forward:
            ref_DXL_raw_pos[set_id] = DXLConstant::MAX_POSITION_VALUE; 
            break;
          case setRotationType::reverse:
            ref_DXL_raw_pos[set_id] = DXLConstant::MIN_POSITION_VALUE;
            break;
          case setRotationType::nomal:
              ref_DXL_raw_pos[set_id] = current_DXL_raw_pos;
              break;
          default:
              ROS_ERROR("this direction of rotation is invalid");
              break;
        }
        if(type == severalType::one){
            for (int i = 1; i < dynamixel_num.size(); ++i) {
                ref_DXL_raw_pos[i] = ref_DXL_raw_pos[0];
            }
        }
        for(auto &DXL : DXLservo){
            DXL.PosDirect();
        }
    }else if(DXL_MODE == DXL::MODE::TORQUE_CONTROL){
        if(type == severalType::one){
            switch(direction){
                case setRotationType::forward:
                  break;
                case setRotationType::reverse:
                   break;
                case setRotationType::nomal:
                   break;
                default:
                   ROS_ERROR("this direction of rotation is invalid");
                   break;
            }
        }else if(type == severalType::all){
            switch(direction){
                case setRotationType::forward:
                    break;
                case setRotationType::reverse:
                    break;
                case setRotationType::nomal:
                    break;
                default:
                    ROS_ERROR("this direction of rotation is invalid");
                    break;
            }
        }
        for(auto &DXL : DXLservo){
          DXL();
        }
    }
}

inline void Rotation::reset() {
  for (int i = 0; i < dynamixel_num.size(); ++i) {
    //TODO:template化
    ref_DXL_rad[i] = degToRad<double>(DXLConstant::ORIGIN_DEG);
  }
}
