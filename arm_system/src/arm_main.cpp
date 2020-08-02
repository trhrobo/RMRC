#include<robot_arm/robot_arm.h>
#include<ros/ros.h>
#include<tuple>
#include<chrono>
#include</usr/local/include/eigen3/Dense>

using namespace Eigen;

int main(int argc, char **argv){
    //腕の長さ
    constexpr double L1 = 100;
    constexpr double L2 = 100;
    constexpr double time_total = 10;
    //スタート位置
    Vector3d pos_start(141.42, 0, 0);
    //移動後の目標座標
    Vector3d pos_end(0, 141.42, 0); 
    ros::init(argc, argv, "robot_arm");
    ros::NodeHandle n;
    ros::Rate loop_rate(400);

    auto timer_start = std::chrono::system_clock::now();

    Arm3Dof::RobotArm<Arm3Dof::RouteMethod::joint_space, Arm3Dof::RouteProfile::lcpb> robotModel(pos_start, pos_end, time_total, L1, L2);

    while(ros::ok()){
        auto timer_now = std::chrono::system_clock::now();
        //経過時間をミリ秒で取得
        double time_elapsed = (std::chrono::duration_cast<std::chrono::milliseconds>(timer_now - timer_start).count()) / 1000.0;
        //間接角を取得
        Vector3d pos_current = robotModel(std::move(time_elapsed));
        std::cout << pos_current * (180.0 / M_PI) << std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }
}