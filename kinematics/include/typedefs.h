#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include "EigenLib.h"
#include "geometry_msgs/Pose.h"
#include <unistd.h> 

/**
 * @file typedefs.h
 * @brief Contains various typedefs, constants, and structures used in the project.
 */

/**
 * @brief 6-dimensional vector of doubles.
 */
typedef Eigen::Matrix<double, 6, 1> Vector6d;

/**
 * @brief 9-dimensional vector of doubles.
 */
typedef Eigen::Matrix<double, 9, 1> Vector9d;

/**
 * @brief 8x6 matrix of doubles.
 */
typedef Eigen::Matrix<double, 8, 6> Matrix86d;

/**
 * @brief 6x8 matrix of doubles.
 */
typedef Eigen::Matrix<double, 6, 8> Matrix68d;

/**
 * @brief 6x6 matrix of doubles.
 */
typedef Eigen::Matrix<double, 6, 6> Matrix66d;

/**
 * @brief Vector representing joint angles 'A' in the Denavit-Hartenberg parameters.
 */
static Vector6d A = {0, -0.425, -0.3922, 0, 0, 0};

/**
 * @brief Vector representing link lengths 'D' in the Denavit-Hartenberg parameters.
 */
static Vector6d D = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};

/**
 * @brief Vector representing link twists 'Alpha' in the Denavit-Hartenberg parameters.
 */
static Vector6d Alpha = {M_PI/2, 0, 0, M_PI/2, -M_PI/2, 0};

/**
 * @brief Default handle position in 3D space.
 */
static Eigen::Vector3d dflHndlPos = {0.561252, 0.715608, 1.27};

/**
 * @brief Secondary handle position in 3D space.
 */
static Eigen::Vector3d dflHndlPos2 = {0.559913, 0.595162, 1.27};

/**
 * @brief Transitional position in 3D space.
 */
static Eigen::Vector3d transitionalPos = {0.497864, 0.618456, 1.27};

/**
 * @brief Castle building point in 3D space.
 */
static Eigen::Vector3d castleBuildingPoint = {0.634939, 0.695369, 1.02};

/**
 * @brief Default resting height.
 */
static double restingH = 1.27;

/**
 * @brief Default working height.
 */
static double workingH = 1.19;

/**
 * @brief Default releasing height.
 */
static double releasingH = 1.023;

/**
 * @brief Order of castle blocks for construction.
 */
static const char* castleBlocksOrder[4] = {"X1-Y3-Z2", "X1-Y2-Z2", "X1-Y1-Z2"};

/**
 * @brief Structure representing direct kinematics information.
 */
typedef struct directK{
    Eigen::Vector3d pe; /**< End-effector position vector. */
    Eigen::Matrix3d Re; /**< End-effector orientation matrix. */
}directK;

/**
 * @brief Structure representing information about a block.
 */
typedef struct blockInfo{
    double x; /**< X-coordinate of the block. */
    double y; /**< Y-coordinate of the block. */
    double z; /**< Z-coordinate of the block. */
    int gripperClosure; /**< Gripper closure value. */
    double zOffset; /**< Offset in the Z direction. */
    double graspingH; /**< Height for grasping the block. */
    double standingGraspingH; /**< Standing height for grasping the block. */
    double yOffset; /**< Offset in the Y direction. */
    const char* classId; /**< Class ID of the block. */
}blockInfo;

/**
 * @brief Structure representing simulated vision information.
 */
typedef struct visionSim{
    geometry_msgs::Pose pose; /**< Pose of the block. */
    double size; /**< Size of the block. */
    double rotx; /**< X-axis rotation of the block. */
    double roty; /**< Y-axis rotation of the block. */
    double rotz; /**< Z-axis rotation of the block. */
    std::string class_id; /**< Class ID of the block. */
}visionSim;

/**
 * @brief Structure representing joint angles and gripper state.
 */
static struct jointsAndGripper{
    Vector6d joints; /**< Joint angles. */
    Eigen::Vector3d gripper; /**< Gripper state. */
}jointsAndGripper;

#endif
