#ifndef DIRECTKINEMATICS_H
#define DIRECTKINEMATICS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

typedef struct directK{
    Eigen::Vector3d pe;
    Eigen::Matrix3d Re;
}directK;

directK ur5DirectKinematics(Eigen::Matrix<double,6,1> Th);

#endif