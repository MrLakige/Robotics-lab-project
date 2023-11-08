#include <iostream>
#include <cmath>
#include <algorithm>
#include "ros/ros.h"
#include "EigenLib.h"
#include "ros_impedance_controller/generic_float.h"
#include "gripper.h"
#include "std_srvs/Trigger.h"


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
    ros::NodeHandle nH;
    ros::ServiceServer moveGripper = nH.advertiseService<ros_impedance_controller::generic_float>("move_gripper", move_gripper_callback);
}
Gripper::~Gripper(){}

void Gripper::move_gripper_callback(ros_impedance_controller::generic_float req){
    diameter = req.data;
    move_gripper(diameter);
}
void Gripper::resend_robot_program(){
    ros::Duration(1.5).sleep();
    //bool ros::service::waitForService	(const std::string & service_name, ros::Duration timeout = ros::Duration(-1))
    ros::Service::waitForService("/ur5/ur_hardware_interface/resend_robot_program");
    ros::ServiceClient sosService = nH.serviceClient<std_srvs::Trigger>("/ur5/ur_hardware_interface/resend_robot_program");
    std_srvs::Trigger srv;  
    bool result = sosService(srv);
    ros::Duration(0.1).sleep();
}

float Gripper::mapToGripperJoints(int diameter){
    if(strcmp(gripperType, "soft_2")==0){
        int D0 = 40;
        int L = 60;
        double delta =0.5*(diameter - D0);
        return atan2(delta, L);
    }else{
        if(strcmp(gripperType, "robotiq_2")==0){
            return 0.8 -(diameter) * 0.8/80.;  // D = 0.1  => 0, D = 0 => 0.8
        }else{
            return (diameter - 22) / (130 - 22) * (-np.pi) + np.pi;  // D = 130-> q = 0, D = 22 -> q = 3.14
        }
    }
}

Eigen::Vector2d Gripper::getDesGripperJoints(){

}

void Gripper::move_gripper(int diameter, std::string status){
    if(!_realRobot){
        float q_finger = mapToGripperJoints(diameter);
        qDesGripper = q_finger * Eigen::Vector3d(1, 1, 1);
        return;
    }

    //skipped real robot implementation 
}