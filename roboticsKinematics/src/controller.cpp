#include <cmath>
#include <algorithm>
#include <numeric>
#include "ros/ros.h"
#include "Eigen/Core"
#include "kinematics.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Pose.h"
#include "controller.h"

/* 
* Robotic arm constuctor, with main features, topics and publisher of the arm 
*/
RoboticArm::RoboticArm(){
    gripperState = 0;
    controllerTopic = "/trajectory_controller";
    default_joint_trajectory.joint_names.resize(6);
    for(int i=0; i<6; i++){
        default_joint_trajectory.joint_names[i] = jointsNames[i];
    }
    JointState = getControllerState(controllerTopic);
    Pose = getPose(JointState.actual.positions).xYZ;
    Eigen::Vector3d pose = Eigen::Vector3d::Map(&Pose[0], 3);
    gripperPose.first = pose;
    Matrix3d PoseQuat = getPose(JointState.actual.positions).rot;
    Eigen::Quaterniond q(PoseQuat);
    gripperPose.second = q;
    jointsPub = nh.advertise<trajectory_msgs::JointTrajectory>(controllerTopic+"/command", 10);
}

/*
* Function to define and move the robotic arm to the desired position
*/
void RoboticArm::move(double dx, double dy, double dz, const Quaterniond& deltaQuat, bool blocking){
    Eigen::Vector3d startingPose = gripperPose.first;
    Quaterniond startingQuaterniond = gripperPose.second;
    double tx, ty, tz;
    tx = startingPose[0] + dx;
    ty = startingPose[1] + dy;
    tz = startingPose[2] + dz;
    Quaterniond targetQuat = startingQuaterniond* deltaQuat;    
    moveTo(tx, ty, tz, targetQuat, blocking);
}

/*
* Function to move the robotic arm to a desired position
*/
void RoboticArm::moveTo(double x, double y , double z, const Quaterniond& targetQuat, double zRaise, bool blocking){
    
    auto smooth = [](double percentValue, double period=M_PI){ //used to make the transition smooth 
        return (1 - std::cos(percentValue * period)) / 2;
    };

    Eigen::Vector3d startingPose = gripperPose.first;
    Quaterniond startingQuaterniond = gripperPose.second;

    if (std::isnan(x)){
        x = startingPose[0];
    }

    if (std::isnan(y)){
        y = startingPose[1];
    }

    if (std::isnan(z)){
        z = startingPose[2];
    }

    //define distnace between starting pose and final pose
    double dx, dy, dz;
    dx = x - startingPose[0];
    dy = y - startingPose[1];
    dz = z - startingPose[2];

    double lenght = std::sqrt(dx*dx + dy*dy + dz*dz)*300 + 80;
    double speed = lenght;

    int steps = static_cast<int>(lenght);
    double step = 1/steps;

    for(double i = 0; i<= 1.0+step; i+=step){
        double i2 = smooth(i, 2*M_PI);
        double i1 = smooth(i);

        Quaterniond grip = startingQuaterniond.slerp(i1, targetQuat); //maintain smooth the rotation movement
        sendJoints(startingPose[0] + i1*dx, startingPose[1]+i1*dy, startingPose[2]+i1*dz + i2*zRaise, grip, 1.0/speed*0.9);
        ros::Duration(1.0/speed).sleep();
    }
    if(blocking){
        waitForPosition(0.005, 0.08);
    }

    //update gripper position
    gripperPose.first = Eigen::Vector3d {x,y,z};
    gripperPose.second = targetQuat;
}

/*
* Function to publish the joint trajectory points 
*/
void RoboticArm::sendJoints(double x, double y, double z, const Quaterniond& quat, double duration){
    std::vector<double> jointState;
    Matrix<double, 6, 1> mat = getJoints(x, y, z, quat.toRotationMatrix());
    Eigen::Map<Eigen::Matrix<double, 6, 1>>(jointState.data(), mat.rows(), 1 ) = mat.col(0);
    
    trajectory_msgs::JointTrajectory trajectory = default_joint_trajectory;

    for(int i = 0; i<2; i++){
        trajectory_msgs::JointTrajectoryPoint points;
        points.positions = jointState;
        points.velocities = {0, 0, 0, 0, 0, 0};
        points.time_from_start = ros::Duration(duration);

        trajectory.points = {points};
        jointsPub.publish(trajectory);
    }
}

/*
* Function to wait if the robotic arm has reached the desired position
*/
void RoboticArm::waitForPosition(double  tollerancePos, double tolleranceVel, double timeout){
    ros::Time endTime = ros::Time::now() + ros::Duration(timeout);
    while(ros::Time::now()< endTime){
        control_msgs::JointTrajectoryControllerState msg = getControllerState(controllerTopic, 10);
        double v = std::accumulate(msg.actual.velocities.begin(), msg.actual.velocities.end(), 0.0);
        if( v < tolleranceVel){
            for(size_t i = 0; i< msg.actual.positions.size(); i++){
                if(std::abs(msg.actual.positions[i] - msg.desired.positions[i])> tollerancePos){
                    break;
                }
                return;
            }
        }
    }
    ROS_WARN("Waiting for position timeout");
}

/*
* Function to wait for a message on ControllerState topic
*/
control_msgs::JointTrajectoryControllerState RoboticArm::getControllerState(std::string controllerTopic, double timeout){
    boost::shared_ptr<control_msgs::JointTrajectoryControllerState const> sharedPtr;
    control_msgs::JointTrajectoryControllerState msg;

    sharedPtr  = ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>(controllerTopic + "/state", ros::Duration(timeout));
    if (sharedPtr == NULL){
        ROS_INFO("No JointTrajectoryControllerState messages received");
    }else{
        msg = *sharedPtr;
    }
    return msg;
}
