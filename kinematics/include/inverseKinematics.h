#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include "typedefs.h"

Matrix68d ur5inverseKinematics(Eigen::Vector3d p60, Eigen::Matrix3d R60);

#endif