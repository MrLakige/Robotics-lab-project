#include <cmath>
#include <algorithm>
#include <numeric>
#include "ros/ros.h"
#include "Eigen/Core"
#include "kinematics.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "controller.h"

/**
 * @brief Constructor for the RoboticArm class.
 * Initializes the arm's features, topics, and publishers.
 */
RoboticArm::RoboticArm(){
    gripperState = 0;
    controllerTopic = "/trajectory_controller";
    default_joint_trajectory.joint_names.resize(6);
    for(int i=0; i<6; i++){
        default_joint_trajectory.joint_names[i] = jointsNames[i];
    }
    JointState = getControllerState(controllerTopic);
    const std::vector<double>& poseData = getPose(JointState.actual.positions).xYZ;
    Eigen::Vector3d pose = Eigen::Vector3d::Map(poseData.data());
    gripperPose.first = pose;
    Matrix3d PoseQuat = getPose(JointState.actual.positions).rot;
    Eigen::Quaterniond q(PoseQuat);
    gripperPose.second = q;
    jointsPub = nh.advertise<trajectory_msgs::JointTrajectory>(controllerTopic+"/command", 10);
}

/**
 * @brief Move the robotic arm by a specified amount.
 * @param dx Change in x-coordinate.
 * @param dy Change in y-coordinate.
 * @param dz Change in z-coordinate.
 * @param deltaQuat Change in orientation as a Quaternion.
 * @param blocking If true, the function will block until movement is complete.
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

/**
 * @brief Move the robotic arm to a desired position.
 * @param x Desired x-coordinate.
 * @param y Desired y-coordinate.
 * @param z Desired z-coordinate.
 * @param targetQuat Target orientation as a Quaternion.
 * @param zRaise Additional z-coordinate raise.
 * @param blocking If true, the function will block until movement is complete.
 */
void RoboticArm::moveTo(double x, double y , double z, const Quaterniond& targetQuat, double zRaise, bool blocking){
    printf("moveto1\n");
    auto smooth = [](double percentValue, double period=M_PI){ //used to make the transition smooth 
        printf("moveto2\n");
        return (1 - std::cos(percentValue * period)) / 2;
    };
    printf("moveto3\n");
    Eigen::Vector3d startingPose = gripperPose.first;
    printf("moveto4\n");
    Quaterniond startingQuaterniond = gripperPose.second;
    printf("moveto5\n");

    if (std::isnan(x)){
        printf("moveto6\n");
        x = startingPose[0];
    }

    if (std::isnan(y)){
        printf("moveto7\n");
        y = startingPose[1];
    }

    if (std::isnan(z)){
        printf("moveto8\n");
        z = startingPose[2];
    }
    printf("moveto9\n");
    //define distnace between starting pose and final pose
    double dx, dy, dz;
    dx = x - startingPose[0];
    dy = y - startingPose[1];
    dz = z - startingPose[2];
    printf("moveto10\n");

    double lenght = std::sqrt(dx*dx + dy*dy + dz*dz)*300 + 80;
    double speed = lenght;

    int steps = static_cast<int>(lenght);
    long double step = 1./steps;
    printf("steps: %Lf\n", step);

    for(double i = 0; i<= 1.0+step; i+=step){
        printf("moveto11\n");
        double i2 = smooth(i, 2*M_PI);
        double i1 = smooth(i);
        printf("moveto12\n");
        Quaterniond grip = startingQuaterniond.slerp(i1, targetQuat); //maintain smooth the rotation movement
        printf("moveto13\n");
        sendJoints(startingPose[0] + i1*dx, startingPose[1]+i1*dy, startingPose[2]+i1*dz + i2*zRaise, grip, 1.0/speed*0.9);
        printf("moveto14\n");
        ros::Duration(1.0/speed).sleep();
        printf("moveto15\n");
    }
    if(blocking){
        waitForPosition(0.005, 0.08);
    }

    //update gripper position
    gripperPose.first = Eigen::Vector3d {x,y,z};
    gripperPose.second = targetQuat;
}

/**
 * @brief Send joint trajectory points to the robotic arm.
 * @param x Desired x-coordinate.
 * @param y Desired y-coordinate.
 * @param z Desired z-coordinate.
 * @param quat Target orientation as a Quaternion.
 * @param duration Duration of the movement.
 */
void RoboticArm::sendJoints(double x, double y, double z, const Quaterniond& quat, double duration){
    printf("sendJ1\n");
    Matrix<double, 6, 1> mat = getJoints(x, y, z, quat.toRotationMatrix());
    printf("sendJ2\n");
    std::vector<double> jointState(mat.data(), mat.data()+mat.size());
    printf("sendJ3\n");
    
    trajectory_msgs::JointTrajectory trajectory = default_joint_trajectory;
    printf("sendJ4\n");

    for(int i = 0; i<2; i++){
        printf("sendJ5\n");
        trajectory_msgs::JointTrajectoryPoint points;
        points.positions = jointState;
        printf("sendJ6\n");
        points.velocities = {0, 0, 0, 0, 0, 0};
        printf("sendJ7\n");
        points.time_from_start = ros::Duration(duration);
        printf("sendJ8\n");

        trajectory.points = {points};
        printf("sendJ9\n");
        jointsPub.publish(trajectory);
        printf("sendJ10\n");
    }
}

/**
 * @brief Wait for the robotic arm to reach the desired position.
 * @param tollerancePos Position tolerance.
 * @param tolleranceVel Velocity tolerance.
 * @param timeout Timeout duration in seconds.
 */
void RoboticArm::waitForPosition(double  tollerancePos, double tolleranceVel, double timeout){
    ros::Time endTime = ros::Time::now() + ros::Duration(timeout);
    printf("%s\n", controllerTopic.c_str());
    while(ros::Time::now()< endTime){
        control_msgs::JointTrajectoryControllerState msg = getControllerState(controllerTopic, 1800);
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

/**
 * @brief Get the current state of the controller.
 * @param controllerTopic Topic of the controller.
 * @param timeout Timeout duration in seconds.
 * @return Controller state message.
 */
control_msgs::JointTrajectoryControllerState RoboticArm::getControllerState(std::string controllerTopic, double timeout){
    boost::shared_ptr<control_msgs::JointTrajectoryControllerState const> sharedPtr;
    control_msgs::JointTrajectoryControllerState msg;
    
    sharedPtr  = ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>(controllerTopic + "/state", ros::Duration(timeout));
    if (sharedPtr == NULL){
        ROS_INFO("No JointTrajectoryControllerState messages received");
        printf("Ok13\n");
    }else{
        printf("nonOk13\n");
        msg = *sharedPtr;
        printf("nonOk14\n");
    }
    return msg;
}
