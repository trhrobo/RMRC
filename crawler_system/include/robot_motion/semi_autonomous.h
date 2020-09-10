 /**
 * @file semi_autonomous.h
 * @brief 半自動制御モード
**/
#pragma once

#include<iostream>
#include<ros/ros.h>
#include<vector>
#include<sensor_msgs/JointState.h>
#include<std_msgs/Float64MultiArray.h>
#include<std_msgs/Float64.h>
#include<memory>
#include"flipper_util.h"
#include"robot_motion/Constant.h"

using std::vector;
//FIXME:定数定義をスコープを付けてまとめる
namespace DXLConstant{
  constexpr int POSE_UP = 90;
  constexpr int POSE_DOWN = -90;
  constexpr int DISTANCE_THRESHOLD_FORWARD = 100;
  constexpr int DISTANCE_THRESHOLD_DOWN = 6;
}

struct DXLPose{
  vector<double> POSE_1{0, 0};
  vector<double> POSE_2{0, 0};
};

struct feedBackTypes{
  //FIXME:unionにして必ず一つの値だけを持つようにする
  //NOTE:ここにフィードバックシステムを追加する
  bool dist;
  bool pos;
  bool torque;
};

template<typename T>
class SemiAutoBase{
  protected:
    ros::Subscriber tof_sub;
    vector<T> current_DXL_pose{0, 0, 0, 0};
    vector<T> current_DXL_load{0, 0, 0, 0};
    vector<T> tof_distance{0, 0, 0, 0};
    vector<T> goal_DXL_pose{0, 0, 0, 0};
    T gyro_pose;
    T goal_pose_right;
    T goal_pose_left;
    const feedBackTypes feedback;
  public:
    SemiAutoBase(ros::NodeHandle _n, feedBackTypes _feedback):feedback(_feedback){
      tof_sub = _n.subscribe("tof_sub", 10, &SemiAutoBase::tofCallback, this);
    }
    void init(){
    }
    void tofCallback(const std_msgs::Float64MultiArray &msg){
      for(int i = 0; i < 4; ++i){
        tof_distance[i] = msg.data[i];
      }
    }
    bool DXLLoad(){
    }
    virtual void operator()(double (&set_array)[4]) = 0;
    virtual T psdCurve() = 0;
};

template<typename T>
class SemiAutoFront : public SemiAutoBase<T>{
  private:
    ros::Subscriber psd_front_sub;
    DXLPose poseParamFront;
  public:
    //FIXME:コンストラクタを修正する(多分こんなに階層的にしないでもいいはずstatic使う？？)
    SemiAutoFront(ros::NodeHandle _n, feedBackTypes _feedback) : SemiAutoBase<T>(_n, _feedback){
      poseParamFront.POSE_1 = {60, 60};
      poseParamFront.POSE_2 = {20, 20};
    }
    //TODO:psdCurveの実装をきちんとする
    T psdCurve(){
      //return -(tof_distance[0] * tof_distance[0] / 250) + 40;
    }
    void operator()(T (&set_array)[4]){
      //TODO:出力値はdeg
      /*
      bool flag_DXL_load = this -> DXLLoad();
      //FIXME:それぞれのフィードバックの紐付けが出来ていない
      if(feedback.dist){
        //NOTE:接地判定 && 閾値以内に障害物があるかどうかの判定
        if(tof_distance[1] < DISTANCE_THRESHOLD_DOWN && tof_distance[0] < DISTANCE_THRESHOLD_FORWARD){
          set_array[0] = this -> poseParamFront.POSE_1[0] - psdCurve();
          set_array[1] = this -> poseParamFront.POSE_1[1] - psdCurve();
        }else{
          set_array[0] = this -> poseParamFront.POSE_2[0];
          set_array[1] = this -> poseParamFront.POSE_2[1];
        }
      }else if(feedback.pos){
        //TODO:各フィードバック系の詳細の追加
        //NOTE:位置(角度)フィードバック
        if(feedback.dist){
          //NOTE:距離、位置(角度)フィードバック
        }
      }else if(feedback.torque){
        //NOTE:トルクフィードバック
        if(feedback.dist && feedback.pos){
          //NOTE:距離、位置(角度)、トルクフィードバック
        }else if(feedback.dist){
          //NOTE:距離、トルクフィードバック
        }else if(feedback.pos){      
          //NOTE:位置(角度)、トルクフィードバック    
        }
      }*/
    }
};

template<typename T>
class SemiAutoRear : public SemiAutoBase<T>{
  private:
    ros::Subscriber psd_rear_sub;
    DXLPose poseParamRear;
  public:
    SemiAutoRear(ros::NodeHandle _n, feedBackTypes _feedback) : SemiAutoBase<T>(_n, _feedback){
      poseParamRear.POSE_1 = {60, 60};
      poseParamRear.POSE_2 = {20, 20};
    }
    T psdCurve(){
    }
    void operator()(T (&set_array)[4]){
      /*
      if(feedback.dist){
          //NOTE:接地判定 && 閾値以内に障害物があるかどうかの判定
        if(tof_distance[1] > DXLConstant::DISTANCE_THRESHOLD_DOWN && tof_distance[3] > DXLConstant::DISTANCE_THRESHOLD_DOWN && tof_distance[0] > DXLConstant::DISTANCE_THRESHOLD_FORWARD){
          set_array[2] = psdCurve();
          set_array[3] = psdCurve();
        }else if(tof_distance[1] > DXLConstant::DISTANCE_THRESHOLD_DOWN|| tof_distance[0] < DXLConstant::DISTANCE_THRESHOLD_FORWARD){
          //変化しない
        }else{
          set_array[2] = this -> poseParamRear.POSE_1[0];
          set_array[3] = this -> poseParamRear.POSE_1[1];
        }    
      }else if(feedback.pos){
        //NOTE:位置(角度)フィードバック
        if(feedback.dist){
          //NOTE:距離、位置(角度)フィードバック
        }  
      }else if(feedback.torque){
        //NOTE:トルクフィードバック
        if(feedback.dist && feedback.pos){
          //NOTE:距離、位置(角度)、トルクフィードバック
        }else if(feedback.dist){
          //NOTE:距離、トルクフィードバック
        }else if(feedback.pos){      
          //NOTE:位置(角度)、トルクフィードバック    
        }
      }*/
    }
};

//FIXME:template継承を間違っているここでtemplate<typename T>するだけでいいのか？？
template<typename T>
class semiAuto{
  private:
    std::unique_ptr<SemiAutoFront<T>> front;
    std::unique_ptr<SemiAutoRear<T>> rear;
  public:
    semiAuto(ros::NodeHandle _n, feedBackTypes _feedback)
      : front(new SemiAutoFront<T>(_n, _feedback)), 
        rear(new SemiAutoRear<T>(_n, _feedback)){}
    void operator()(T (&set_array)[4]){
      (*front)(set_array);
      (*rear)(set_array);
    }
};