#ifndef semi_autonomous
#define semi_autonomous

#define POSE_UP 90;
#define POSE_DOWN -90;
#define DISTANCE_THRESHOLD 10;
#define POSE_1 1;
#define POSE_2 2;

typedef struct{
  vector<double> POSE_1{0, 0};
  vector<double> POSE_2{0, 0};
}dynamixelPose;

class SemiAutonomousBase{
  protected:
    ros::Publisher dynamixel_pub;
    ros::Subscriber dynamixel_sub;
    ros::Subscriber gyro_sub;
    ros::Subscriber tof_sub;
    vector<double> current_dynamixel_pose{0, 0, 0, 0};
    vector<double> current_dynamixel_load{0, 0, 0, 0};
    vector<double> tof_distance{0, 0};
    vector<double> goal_dynamixel_pose{0, 0, 0, 0};
    double gyro_pose;
    double goal_pose;
  public:
    SemiAutonomousBase(){
      ros::NodeHandle n;
      dynamixel_pub = n.advertise<std_msgs::>;
      dynamixel_sub = n.subscribe("dynamixel_sub", 10, &SemiAutonomous::dynamixelCallback, this);
      gyro_sub = n.subscribe("gyro_sub", 10, &SemiAutonomous::gyroCallback, this);
      tof_sub = n.subscribe("tof_sub", 10, &SemiAutonomous::tofCallback, this);
    }
    void init();

    void dynamixelCallback(const sensor_msgs::JointState &jointstate){
      for(int i = 0; i < 4; ++i){
        current_dynamixel_pose[i] = jointstate[3 - i];
        current_dynamixel_load[i] = jointstate[3 - i];
      }
    }

    void gyroCallback(const std_msgs::Float64 &msg){
      gyro_pose = msg.data;
    }

    void tofCallback(const std_msgs::Float64MultiArray &msg){
      for(int i = 0; i < 2; ++i){
        tof_distance[i] = msg.data[i];
      }
    }

    bool dynamixelLoad(){
    }

    virtual void mainSemiAutonomous() = 0;
};

class SemiAutonomousFront : public SemiAutonomousBase{
  private:
    dynamixelPose poseParamFront;
  public:
    SemiAutonomousFront{
      poseParamFront.POSE_1{1, 1};
      poseParamFront.POSE_2(2, 2);
    }
    void mainSemiAutonomous() override{
      bool flag_dynamixel_load = this -> dynamixelLoad();
      if(tof_distance < DISTANCE_THRESHOLD && dynamixelLoad == true){
        if(gyro_pose > POSE_UP){
          goal_pose = this -> poseFront.POSE_1;
        }else{
          goal_pose = this -> poseFront.POSE_2;
        }
      }
    }
};

class SemiAutonomousRear : public SemiAutonomousBase{
  private:
    dynamixelPose poseParamRear;
  public:
    SemiAutonomousRear(){
      poseParamRear.POSE_1(3, 3);
      poseParamRear.POSE_2(4, 4);
    }
    void mainSemiAutonomous() override{
    }
};

class semiAutonomous{
  private:
    SemiAutonomousFront *front;
    SemiAutonomousRear *rear;
  public:
    semiAutonomous(){
      front = new SemiAutonomousFront;
      rear = new SemiAutonomousRear;
    }
    ~semiAutonomous(){
      delete front;
      delete rear;
    }
    void main(){
      front -> mainSemiAutonomous;
      rear -> mainSemiAutonomous;
    }
};
#endif semi_autonomous
