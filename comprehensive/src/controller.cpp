#include <comprehensive/Button.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

comprehensive::Button data;

namespace dozap {
double map(double x, double in_min, double in_max, double out_min,
           double out_max) {
  return (double)(x - in_min) * (out_max - out_min) / (in_max - in_min) +
         out_min;
}
}

void joyCallback(const sensor_msgs::Joy &msg) {
  data.move = hypot(msg.axes[1], msg.axes[0]) * 255;
  data.move_angle = atan2(msg.axes[1], msg.axes[0]);
  data.rotation_right = dozap::map(msg.axes[5], 1, -1, 0, 255);
  data.rotation_left = dozap::map(msg.axes[2], 1, -1, 0, 255);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "xbox");
  ros::NodeHandle n;
  ros::Publisher controller_pub =
      n.advertise<comprehensive::Button>("xbox", 10);
  ros::Subscriber controller_sub = n.subscribe("joy", 10, joyCallback);
  ros::Rate loop_rate(100);

  while (ros::ok()) {
    controller_pub.publish(data);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
