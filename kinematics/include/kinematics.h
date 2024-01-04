#ifndef KINEMATICS_H
#define KINEMATICS_H


#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <realtime_tools/realtime_publisher.h>
#include "typedefs.h"

static struct jointsAndGripper{
    Vector6d joints;
    Eigen::Vector3d gripper;
}jointsAndGripper;

// Methods
void sendDesJstate(const Vector6d & joint_pos, const Eigen::Vector3d & gripper_pos);
void readJoints();
void moveGripper(const double diameter);
double mapToGripperJoints(double diameter);
void setNewGripperPosition(double diameter);
void move_gripper(double diameter);
void setJoints(Vector6d q);
void setGripper(Eigen::Vector3d gripper);
void setJointsAndGripper(Vector6d q, Eigen::Vector3d gripper);
void move(Eigen::Vector3d poseF, Eigen::Vector3d orientF);

//void initFilter(const Vector6d & joint_pos);
//Vector6d secondOrderFilter(const Vector6d & input, const double rate, const double settling_time);

static double loop_time = 0.;
static double loop_frequency = 250.;

// Publishers
static ros::Publisher pub;
static ros::ServiceClient gripperClient;
//sensor_msgs::JointState jointState_msg_sim;
static std_msgs::Float64MultiArray jointState_msg_sim;
static std_msgs::Float64MultiArray jointState_msg_robot;

static bool real_robot = false;
static bool soft_gripper = false;
static bool gripper_sim = false;

#endif