#include <iostream>
#include "Eigen/Dense"
#include <cmath>

using namespace Eigen;
  
const double d[6] = [0.089159,  0.00000,  0.00000,  0.10915,  0.09465,  0.0903 + 0.1628];
const double a[6] = [0.00000,  -0.42500, -0.39225,  0.00000,  0.00000,  0.0000];
const double  alph[6] = [pi/2,      0,        0,        pi/2,     -pi/2,    0];
#ifndef KINEMATICS_H
#define KINEMATICS_H

Matrix4d AH(int n, double th[6]);

Matrix4d forwardKinematics(double th[6]);

Matrix<double, 6, 8> inverseKinematics(Matrix4d& desired_position);

Matrix<double, 6, 1> getJoints(double x, double y, double z, Matrix3d rot);

struct returnValues{
    double x;
    double y;
    double z;
    Matrix3d rot;
}retV;

returnValues getPose(double joints[6]);

#endif