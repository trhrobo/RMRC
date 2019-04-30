#include<ros/ros.h>
#include<dozap_second/Button.h>
#include<sensor_msgs/Joy.h>
#include<geometry_msgs/Twist.h>

dozap_second::Button data;
//sensor_msgs::Joy joy;
geometry_msgs::Twist cmd_vel;

int move, rotation_right, rotation_left;

/*void joy_callback(const sensor_msgs::Joy &joy_msg){ 
     cmd_vel.linear.x =joy_msg.axes[1];
     cmd_vel.linear.y =joy_msg.axes[0];
     cmd_vel.angular.z=joy_msg.axes[2];
}
*/     
void joy_callback(const sensor_msgs::Joy &joy_msg){
     move = (int)joy_msg.axes[3];
     rotation_right = joy_msg.buttons[1];
     rotation_left = joy_msg.buttons[2];
}

int main(int argc, char **argv){
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;
    
//    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    ros::Publisher controller_pub = n.advertise<dozap_second::Button>("controller_info", 1000);
    ros::Subscriber joy_sub = n.subscribe("joy", 10, joy_callback);
    ros::Rate loop_rate(10);

    while(ros::ok()){
	data.move = move;
	data.rotation_right = rotation_right;
        data.rotation_left = rotation_left;
        controller_pub.publish(data);
      //  cmd_pub.publish(cmd_vel);
       // ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

