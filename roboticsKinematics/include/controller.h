#ifndef CONTOLLER_H
#define CONTOLLER_H

#include <cmath>
#include <algorithm>
#include "ros/ros.h"
#include "EigenLib.h"
#include "kinematics.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Pose.h"

class RoboticArm {

public:

    RoboticArm();

    void move(double dx=0, double dy=0, double dz=0, const Eigen::Quaterniond& deltaQuat=Quaterniond(1, 0, 0, 0), bool blocking=true);

    void moveTo(double x = std::nan(""), double y = std::nan(""), double z = std::nan(""), const Eigen::Quaterniond& targetQuat = Quaterniond(1, 0, 0, 0), double zRaise = 0.0, bool blocking = true);

    void sendJoints(double x, double y, double z, const Eigen::Quaterniond& quat, double duration=1.0);

    void waitForPosition(double  tollerancePos=0.01, double tolleranceVel=0.01, double timeout=2.0);

    control_msgs::JointTrajectoryControllerState getControllerState(std::string controllerTopic, double timeout=0);

    double gripperState;
    std::string controllerTopic;
    std::string jointsNames[6]  = {
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint"
    };
    trajectory_msgs::JointTrajectory default_joint_trajectory;
    control_msgs::JointTrajectoryControllerState JointState;
    std::vector<double> Pose;
    std::pair<Eigen::Vector3d, Quaterniond> gripperPose;
    ros::NodeHandle nh;
    ros::Publisher jointsPub;

};

#endif
