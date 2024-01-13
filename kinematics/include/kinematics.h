#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "typedefs.h"
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>

static bool real_robot = false;
static bool soft_gripper = false;
static bool gripper_sim = false;

static ros::Publisher pub;
static ros::ServiceClient gripperClient;
static std_msgs::Float64MultiArray jointState_msg_sim;
static std_msgs::Float64MultiArray jointState_msg_robot;

static ros::ServiceClient attach_client;
static ros::ServiceClient detach_client;

static double loop_time = 0.;
static double loop_frequency = 250.;

void sendDesJstate(const Vector6d & joint_pos, const Eigen::Vector3d & gripper_pos);

#endif