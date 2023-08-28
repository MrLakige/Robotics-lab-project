#include <iostream>
#include "Eigen/Dense"
#include <cmath>

using namespace Eigen;

#ifndef KINEMATICS_H
#define KINEMATICS_H

const double d[6] = {0.089159,  0.00000,  0.00000,  0.10915,  0.09465,  0.0903 + 0.1628};
const double a[6] = {0.00000,  -0.42500, -0.39225,  0.00000,  0.00000,  0.0000};
const double  alph[6] = {M_PI/2,      0,        0,        M_PI/2,     -M_PI/2,    0};

Matrix4d AH(int n, VectorXd& th);

Matrix4d forwardKinematics(double th[6]);

Matrix<double, 6, 8> inverseKinematics(Matrix4d& desired_position);

Matrix<double, 6, 1> getJoints(double x, double y, double z, Matrix3d rot);

struct returnValues{
    std::vector<double> xYZ;
    Matrix3d rot;
};

returnValues getPose(std::vector<double> joints);

#endif