#include<ros/ros.h>
#include<Dozap/msg/Plan.h>

Dozap::Plan data;

void motor_callback(const Dozap::button &point){
    if(point.move){
        data.motor_right = (int)point.move;
        data.motor_left = (int)point.move;
    }else if(point.rotation_right){
        data.motor_right = (int)point.rotation_right;
        data.motor_left = -(int)point.rotation_right;
    }else if(point.rotation_left){
        data.motor_right = -(int)point.rotation_left;
        data.motor_left = (int)point.rotation_left;
    }else{
        data.motor_right = 0;
        data.motor_left = 0;
    }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "plan");
  ros::NodeHandle n;
  ros::Publisher plan_pub = n.advertise<Dozap::Plan>("motor_info", 1000);
  ros::Subscriber controller_sub = n.advertise("controller_info", 1000, motor_callback);
  ros::Rate loop_rate(10);

  while(ros::ok()){
      plan_pub.publish(data);
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
}
