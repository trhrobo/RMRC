#include<ros/ros.h>
#include<Dozap/msg/Plan.h>

void controller_callback(const Dozap::Plan & )
int main(int argc, char **argv){
  ros::init(argc, argv, "plan");
  ros::NodeHandle n;
  ros::Publisher plan_pub = n.advertise<Dozap::Plan>("Plan_info", 1000);
  ros::Subscriber controller_sub = n.advertise("controller_info", 1000, controller_callback);

