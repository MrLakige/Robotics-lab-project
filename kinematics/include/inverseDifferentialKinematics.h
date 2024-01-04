#ifndef INVERSEDIFFERENTIALKINEMATICS_H
#define INVERSEDIFFERENTIALKINEMATICS_H

#include "typedefs.h"

Eigen::MatrixXd inverseDiffKinematics(Eigen::VectorXd th, Eigen::Vector3d endPos, Eigen::Vector3d endOrientation);

Eigen::MatrixXd inverseDiffKinematicsControlComplete(Vector6d th, Eigen::Vector3d endPos, Eigen::Vector3d endOrientation);

Eigen::MatrixXd inverseDiffKinematicsControlCompleteAnglesAxis(Vector6d th, Eigen::Vector3d endPos, Eigen::Vector3d endOrientation);

#endif