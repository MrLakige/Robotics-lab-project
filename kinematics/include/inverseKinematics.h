#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include "typedefs.h"

/**
 * @brief  Comutation of the inverse kinematics for the UR5 robotic arm.
 *
 * @param p06 The vector x3 with the end effector position.
 * @param R60 MAtrix 3x3 with the end effector rotation.
 * @return MAtrix6x8 with the 8 possible solution of the computation.
 */
Matrix68d ur5inverseKinematics(Eigen::Vector3d p60, Eigen::Matrix3d R60);

#endif