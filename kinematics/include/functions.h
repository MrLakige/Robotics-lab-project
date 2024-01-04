#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "typedefs.h"

const double deltaT = 0.001;

Matrix66d jacobian(Vector6d th);

Matrix66d pseudoInverse(Matrix66d J);

Eigen::VectorXd dotQ(Vector6d qk, Eigen::Vector3d xe, Eigen::Vector3d xd, Eigen::Matrix3d Re, Eigen::Vector3d phid);

Vector6d dotQControlComplete(Vector6d q, Eigen::Vector3d xe, Eigen::Vector3d xd, Eigen::Vector3d vd, Eigen::Matrix3d Re, Eigen::Vector3d phid, Eigen::Vector3d phidDot);

Eigen::Vector3d computeOrientationErrorW(Eigen::Matrix3d w_R_e, Eigen::Matrix3d w_R_d);

Eigen::Vector3d computePositionError(Eigen::Vector3d xe, Eigen::Vector3d xd);

Eigen::VectorXd qDott0(Eigen::VectorXd qk);

Eigen::Vector3d rotmToAngleAxis(Eigen::Matrix3d R);

Eigen::Matrix3d eulerToRotationMatrix(Eigen::Vector3d euler);

Eigen::Vector3d phid(double t, Eigen::Vector3d phief, Eigen::Vector3d phie0);

Eigen::Vector3d pd(double t, Eigen::Vector3d xef, Eigen::Vector3d xe0);

Eigen::Vector3d attrForce_pos(Eigen::Vector3d error);

Eigen::Vector3d repulForce(Eigen::Vector3d xe);

Eigen::Vector3d desPos(Eigen::Vector3d xe, Eigen::Vector3d xf);

Vector6d dotQControlCompleteAangleAxis(Vector6d q, Eigen::Vector3d xe, Eigen::Vector3d xd, Eigen::Vector3d vd, Eigen::Matrix3d Re, Eigen::Vector3d phid, Eigen::Vector3d phidDot);

#endif