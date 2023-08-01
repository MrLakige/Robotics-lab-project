#ifndef CONTOLLER_H
#define CONTOLLER_H

#include <cmath>
#include <algorithm>
#include "ros/ros.h"
#include "Eigen/Core"
#include "kinematics.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Pose.h"

class RoboticArm {

public:

    RoboticArm(gripperState, controllerTopic);

    void move(double dx=0, double dy=0, double dz=0, const Quaternion& deltaQuat=Quaternion(1, 0, 0, 0), bool blocking=true);

    void moveTo(double x = std::nan(""), double y = std::nan(""), double z = std::nan(""), const Quaternion& targetQuat = Quaternion(1, 0, 0, 0), double zRaise = 0.0, bool blocking = true);

    void sendJoints(double x, double y, double z, const Quaternion& quat, double duration=1.0);

    void waitForPosition(double  tollerancePos=0.01, double tolleranceVel=0.01, double timeout=2.0);

private:

    double gripperState;
    std::string controllerTopic;
    std::vector<std::string> jointsNames;
    trajectory_msgs::JointTrajectory default_joint_trajectory;
    std::pair<Eigen::Vector3d, Quaternion> gripperPose;
    ros::NodeHandle nh;
    ros::Publisher joinsPub;

    control_msgs::JointTrajectoryControllerState getControllerState(const std::string& controllerTopic, double timeout=0);
};

#endif