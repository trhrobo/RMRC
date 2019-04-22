#include<ros/ros.h>
#include<pigpiod_if2.h>
#include<Dozap/msg/Motor.h>

void md_callback{

int main(int argc, char **argv){

  int pi;
  pi = pigpio_start(NULL,"8888");
  set_mode(pi, 1, 1);
  set_mode(pi, 2, 1);
  set_mode(pi, 3, 1);
  set_mode(pi, 4, 1);
  set_mode(pi, 5, 1);
  set_mode(pi, 6, 1);
  set_mode(pi, 7, 1);
  set_mode(pi, 8, 1);

  ros::init(argc, argv, "md");
  ros::Nodehandle n;
  ros::Subscriber motor_sub = n.subscribe("motor_info", 1000, md_callback);
  ros::Rate loop_rate(10);

  while(ros::ok()){

