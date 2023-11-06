#include <iostream>
#include <cmath>
#include <algorithm>
#include "ros/ros.h"
#include "EigenLib.h"
#include "ros_impedance_controller/generic_float.h"
#include "gripper.h"


Gripper::Gripper(std::string _gripperType, bool _realRobot, float _dt, float _gripperDuration){
    gripperDuration = _gripperDuration;
    realRobot=_realRobot;
    gripperType = _gripperType;
    if(strcmp(gripperType, "soft_2")==0){
        std::cout<<"\033[0;34m Using soft 2 finger gripper!"<<endl;   
        numberOfFinger=2;
        qDesGripper=Eigen::Vector3d(0.0, 0.0, 0.0);
    }else if(strcmp(gripperType, "robotiq_2")==0){
        std::cout<<"\033[0;34m Using robotiq gripper!"<<endl;   
        numberOfFinger=1;
        qDesGripper=Eigen::Vector3d(0.0, 0.0, 0.0);
    }else{
        std::cout<<"\033[0;34m Using hard 3 finger gripper!"<<endl;   
        numberOfFinger=3;
        qDesGripper=Eigen::Vector3d(1.8, 1.8, 1.8);
    }
    // to complete to make it works
    //ros::ServiceClient moveGripper = nH.serviceClient<ros_impedance_controller::generic_float>("move_gripper");
}
Gripper::~Gripper(){}

void Gripper::move_gripper_callback(ros_impedance_controller::generic_float req){
    diameter = req.data;
    move_gripper(diameter);
}
void Gripper::resend_robot_program(){
    ros::Duration(1.5).sleep();
}

float Gripper::mapToGripperJoints(int diameter);
Eigen::Vector2d Gripper::getDesGripperJoints();
void Gripper::move_gripper(int diameter, std::string status);