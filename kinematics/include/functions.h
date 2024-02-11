#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "typedefs.h"

const double deltaT = 0.001;

/**
 * @brief  Comutation of the jacobian of the UR5 robotic arm.
 *
 * @param Th The vector of joint angles.
 * @return Matrix 6x6 with the result fo the computation.
 */
Matrix66d jacobian(Vector6d th);

/**
 * @brief  Comutation of the pseudo inverse of the UR5 robotic arm.
 *
 * @param J The jacobian of the UR5 robotic arm.
 * @return Matrix 6x6 with the result fo the computation.
 */
Matrix66d pseudoInverse(Matrix66d J);

    /**
     * @brief Computation of the derivative of joint angles with a proportional control.
     * 
     * @param qk Actual joint cinfiguration
     * @param xe Actual end effector position
     * @param xd Desired end effector position
     * @param Re Acual end effector orentation
     * @param phid Desired end effector orientation
     * @return VectorXd With the computed derivative of joint angles
     */
Eigen::VectorXd dotQ(Vector6d qk, Eigen::Vector3d xe, Eigen::Vector3d xd, Eigen::Matrix3d Re, Eigen::Vector3d phid);

/**
 * @brief  Computation of the derivative for to get controlled values for the complete motion.
 *
 * @param q Actual joint cinfiguration.
 * @param xe Actual end effector position.
 * @param xd Desired end effector position
 * @param vd Desired velocity of the end effector position
 * @param Re Acual end effector orentation
 * @param phid Desired end effector orientation
 * @param phidDot Desired angular velocity of the end effector orientation
 * @return Vector x6 with the result fo the computation.
 */
Vector6d dotQControlComplete(Vector6d q, Eigen::Vector3d xe, Eigen::Vector3d xd, Eigen::Vector3d vd, Eigen::Matrix3d Re, Eigen::Vector3d phid, Eigen::Vector3d phidDot);

/**
 * @brief  Comutation of the rotation error.
 *
 * @param w_R_e Matrix 3x3 with the actual end effector rotation.
 * @param w_R_d Matrix 3x3 with the desired end effector rotation.
 * @return Vector with the result fo the computation.
 */
Eigen::Vector3d computeOrientationErrorW(Eigen::Matrix3d w_R_e, Eigen::Matrix3d w_R_d);

/**
 * @brief  Comutation of the posiiton error.
 *
 * @param w_R_e Vector x3 with the actual end effector position.
 * @param w_R_d Matrix x3 with the desired end effector posiiton.
 * @return Vector with the result fo the computation.
 */
Eigen::Vector3d computePositionError(Eigen::Vector3d xe, Eigen::Vector3d xd);


/**
 * @brief  Comutation of the transformation from the rotation matrix to the euler angles.
 *
 * @param R Matrix 3x3 with the roatation matrix.
 * @return Vector x3 with the computed euler angles.
 */
Eigen::Vector3d rotmToAngleAxis(Eigen::Matrix3d R);

/**
 * @brief  Comutation of the transformation from the euler angles to the rotation matrix.
 *
 * @param euler Vector x3 with the computed euler angles.
 * @return Matrix 3x3 with the roatation matrix.
 */
Eigen::Matrix3d eulerToRotationMatrix(Eigen::Vector3d euler);

/**
 * @brief  Comutation of the desired angular velocity for a dt.
 * 
 * @param t current time stamp.
 * @param phief Vector x3 Final orientation of the end effector.
 * @param phie0 Vector x3 Intiial orientation of the end effector.
 * @return Vector x3 The computed angular velocity.
 */
Eigen::Vector3d phid(double t, Eigen::Vector3d phief, Eigen::Vector3d phie0);

/**
 * @brief  Comutation of the desired linear velocity for a dt.
 * 
 * @param t current time stamp.
 * @param xef Vector x3 Final position of the end effector.
 * @param xe0 Vector x3 Intiial position of the end effector.
 * @return Vector x3 The computed linear velocity.
 */
Eigen::Vector3d pd(double t, Eigen::Vector3d xef, Eigen::Vector3d xe0);

/**
 * @brief  Comutation of the attractive force based on the normalized position error
 *
 * @param error Error computed between the actual and desired position of the end effector.
 * @return Vector x3 with the result of the computation of the attarctive force.
 */
Eigen::Vector3d attrForce_pos(Eigen::Vector3d error);

/**
 * @brief  Comutation of the repulsive force in order to keep the end effector away from the shoulder singularity.
 *
 * @param xe actual end effector posiiton.
 * @return Vector x3 with the result of the computation of the repulsive force.
 */
Eigen::Vector3d repulForce(Eigen::Vector3d xe);

/**
 * @brief  Comutation of the desired position of the end effector after computing the attractive and the repulsive forces.
 * 
 * @param xe actual end effector posiiton.
 * @param xf desired end effector posiiton.
 * @return Vector x3 with the result of the computation of the function.
 */
Eigen::Vector3d desPos(Eigen::Vector3d xe, Eigen::Vector3d xf);


/**
 * @brief  Comutation of the transformation from the rotation matrix to the euler angles.
 *
 * @param R Matrix 3x3 with the roatation matrix.
 * @return Vector x3 with the computed euler angles.
 */
Eigen::Vector3d rotMatToEuler(Eigen::Matrix3d rotMatrix);

//void straightenBlockUp(Eigen::Vector3d pos, Eigen::Vector3d rot, std::string blockName);

#endif
