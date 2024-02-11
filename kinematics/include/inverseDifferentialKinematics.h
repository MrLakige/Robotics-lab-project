#ifndef INVERSEDIFFERENTIALKINEMATICS_H
#define INVERSEDIFFERENTIALKINEMATICS_H

#include "typedefs.h"

/**
 * @brief Function that compute the inverse differential kinematics for UR5 robotic arm
 * 
 * @param th Actual joints configuration of the robot
 * @param endPos Deisred target position of the end effector
 * @param endOrientation Deisred target rotation of the end effector
 * @return MatrixXd filled with the via points (joint configurations) to achieve the desired position and orientation
 */
Eigen::MatrixXd inverseDiffKinematics(Eigen::VectorXd th, Eigen::Vector3d endPos, Eigen::Vector3d endOrientation);

/**
 * @brief Function that compute the inverse differential kinematics for UR5 robotic arm for a complete control motion
 * 
 * @param th Actual joints configuration of the robot
 * @param endPos Deisred target position of the end effector
 * @param endOrientation Deisred target rotation of the end effector
 * @return MatrixXd filled with the via points (joint configurations) to achieve the desired position and orientation
 */
Eigen::MatrixXd inverseDiffKinematicsControlComplete(Vector6d th, Eigen::Vector3d endPos, Eigen::Vector3d endOrientation);

/**
 * @brief Function that compute the inverse differential kinematics for UR5 robotic arm for a complete anglular axis motion
 * 
 * @param th Actual joints configuration of the robot
 * @param endPos Deisred target position of the end effector
 * @param endOrientation Deisred target rotation of the end effector
 * @return MatrixXd filled with the via points (joint configurations) to achieve the desired position and orientation
 */
Eigen::MatrixXd inverseDiffKinematicsControlCompleteAnglesAxis(Vector6d th, Eigen::Vector3d endPos, Eigen::Vector3d endOrientation);

#endif