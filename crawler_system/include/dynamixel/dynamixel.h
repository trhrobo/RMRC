#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#define DYNAMIXEL_RESOLUTION 4096
#define DYNAMIXEL_RESOLUTION_ANGLE 0.088

template <class T> class dynamixel{
  private:
    int _id;
  public:
    dynamixel(int user_id);
    ~dynamixel();
    //goal_posの範囲は0-360まで
    int dynamixelSet(T goal_angle, T now_pos);
    T dynamixelTimer();
    T dynamixelReset();
    T angleCal(T goal_value);
    int torqueFB(int goal_pos);
};

#endif

