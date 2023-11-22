#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

Eigen::Matrix<double, 8, 6> ur5inverseKinematics(Eigen::Vector3d p60, Eigen::Matrix3d R60);

#endif
