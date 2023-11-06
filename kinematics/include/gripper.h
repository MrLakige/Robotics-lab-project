#ifndef __GRIPPER_H__
#define __GRIPPER_H__

#include <iostream>
#include <cmath>
#include <algorithm>
#include "ros/ros.h"
#include "EigenLib.h"
#include "ros_impedance_controller/generic_float.h"

class Gripper{
    private:
        bool realRobot;
        std::string gripperType;
        float dt;
        float gripperDuration;
        Eigen::Vector3d qDesGripper;
        int numberOfFinger;

        // filter
        Eigen::Vector3d filter1;
        Eigen::Vector3d filter2;

        Eigen::Vector3d filter(Eigen::Vector3d input, float settlingTime){
            float gain= dt/(0.1 * settlingTime+ dt);
            filter1=qDesGripper;
            filter2=qDesGripper;
            for(int i=0; i<numberOfFinger; i++){
                filter1(i)=(1-gain)*filter1(i)+gain * input(i);
                filter2(i)=(1-gain)*filter2(i)+gain * filter1(i);
            }
            return filter2;
        }


    public: 
        Gripper(std::string _gripperType, bool _realRobot=false, float _dt=0.0001, float _gripperDuration=5.0);
        ~Gripper();
        void move_gripper_callback(ros_impedance_controller::generic_float req);
        void resend_robot_program();
        float mapToGripperJoints(int diameter);
        Eigen::Vector2d getDesGripperJoints();
        void move_gripper(int diameter, std::string status);
}

#endif
