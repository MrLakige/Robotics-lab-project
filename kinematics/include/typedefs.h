#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include "EigenLib.h"

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 8, 6> Matrix86d;
typedef Eigen::Matrix<double, 6, 8> Matrix68d;
typedef Eigen::Matrix<double, 6, 6> Matrix66d;

static Vector6d A = {0, -0.425, -0.3922, 0, 0, 0};
static Vector6d D = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};
static Vector6d Alpha = {M_PI/2,0,0,M_PI/2,-M_PI/2,0};

typedef struct directK{
    Eigen::Vector3d pe;
    Eigen::Matrix3d Re;
}directK;

#endif