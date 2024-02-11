#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include "EigenLib.h"
#include "geometry_msgs/Pose.h"
#include <unistd.h> 

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 8, 6> Matrix86d;
typedef Eigen::Matrix<double, 6, 8> Matrix68d;
typedef Eigen::Matrix<double, 6, 6> Matrix66d;

static Vector6d A = {0, -0.425, -0.3922, 0, 0, 0};
static Vector6d D = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};
static Vector6d Alpha = {M_PI/2,0,0,M_PI/2,-M_PI/2,0};

static Eigen::Vector3d dflHndlPos = {0.561252, 0.715608, 1.27};
static Eigen::Vector3d dflHndlPos2 = {0.559913, 0.595162, 1.27};
static Eigen::Vector3d transitionalPos = {0.497864, 0.618456, 1.27};
static Eigen::Vector3d castleBuildingPoint = {0.634939, 0.695369, 1.02};

static double restingH = 1.27;
static double workingH = 1.19;
static double releasingH = 1.023;

static const char* castleBlocksOrder[4] = {/* "X1-Y4-Z2", */ "X1-Y3-Z2", "X1-Y2-Z2", "X1-Y1-Z2"};

typedef struct directK{
    Eigen::Vector3d pe;
    Eigen::Matrix3d Re;
}directK;

typedef struct blockInfo{
    double x;
    double y;
    double z;
    int gripperClosure;
    double zOffset;
    double graspingH;
    double standingGraspingH;
    double yOffset;
    const char* classId;
}blockInfo;

typedef struct visionSim{
    geometry_msgs::Pose pose;
    double size;
    double rotx;
    double roty;
    double rotz;
    std::string class_id;
}visionSim;

static struct jointsAndGripper{
    Vector6d joints;
    Eigen::Vector3d gripper;
}jointsAndGripper;

#endif