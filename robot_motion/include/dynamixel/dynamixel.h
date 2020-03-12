#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#define DYNAMIXEL_RESOLUTION 4096
#define DYNAMIXEL_RESOLUTION_ANGLE 0.088

class dynamixel{
private:
    int _id;
public:
    dynamixel(int user_id);
    ~dynamixel();
    //goal_posの範囲は0-360まで
    int dynamixelSet(double goal_angle, double now_pos);
    double dynamixelTimer();
    double dynamixelReset();
    double angleCal(double goal_value);
};

#endif

