#include<ros/ros.h>
#include<sensor_msgs/Imu.h>
#include<geometry_msgs/PoseStamped.h>

ros::Publisher posestamped_pub;

void gy521_callback(const sensor_msgs::Imu &imu_msg){
  geometry_msgs::PoseStamped ps_msg;
  ps_msg.header=imu_msg.header;
  ps_msg.pose.orientation = imu_msg.orientation;
  posestamped_pub.publish(ps_msg);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "gy521_pose");
  ros::NodeHandle n;

  posestamped_pub = n.advertise<geometry_msgs::PoseStamped>("imu_pose", 10);
  ros::Subscriber gy521_sub = n.subscribe("gyro_info", 10, gy521_callback);
  ros::spin();
}

