#include<ros/ros.h>
#include<dozap_second/Button.h>
#include<sensor_msgs/Joy.h>

dozap_second::Button data;

void joy_callback(const sensor_msgs::Joy &joy_msg){
    data.move = joy_msg.axes[1];
    data.rotation_right = joy_msg.axes[5];
    data.rotation_left = joy_msg.axes[2];
     
}

int main(int argc, char **argv){
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;
    

    ros::Publisher controller_pub = n.advertise<dozap_second::Button>("controller_info", 1000);
    ros::Subscriber joy_sub = n.subscribe("joy", 10, joy_callback);
    ros::Rate loop_rate(10);

    while(ros::ok()){
	/*data.move = 1;
	data.rotation_right = 1;
        data.rotation_left = 1;
        */
        controller_pub.publish(data);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

