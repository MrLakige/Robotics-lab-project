#include <iostream>
#include "Eigen/Dense"
#include <cmath>
#include "kinematics.h"

using namespace Eigen;

/********************** FORWARD KINEMATICS **********************/

/**
 * @brief  Compute the homogeneous transformation matrix of the N-th joint of the Robotic arm (using the Denavit-Hartenberg transformation).
 *
 * @param n The joint index (1-based) for which to compute the transformation matrix.
 * @param th The vector of joint angles.
 * @return The transformation matrix for the N-th joint.
 */
Matrix4d AH(int n, VectorXd& th){
    Matrix4d T_a = Matrix<double, 4, 4>::Identity();
    T_a(0, 3) = a[n-1];

    Matrix4d T_d = Matrix<double, 4, 4>::Identity();
    T_d(2, 3) = d[n-1];

    Matrix4d Rzt { 
                {cos(th[n - 1]), -sin(th[n - 1]), 0, 0},
                {sin(th[n - 1]), cos(th[n - 1]),  0, 0},
                {0,              0,               1, 0},
                {0,              0,               0, 1}
    };

    Matrix4d Rxa {
                {1, 0,                0,                 0},
                {0, cos(alph[n - 1]), -sin(alph[n - 1]), 0},
                {0, sin(alph[n - 1]), cos(alph[n - 1]),  0},
                {0, 0,                0,                 1}
    };

    return T_d * Rzt * T_a * Rxa;
}

/**
 * @brief Compute the forward kinematics of the robotic arm, allowing retrieval of the gripper position with respect to the base.
 *
 * @param th An array of 6 joint angles.
 * @return The transformation matrix representing the gripper's position and orientation.
 */
Matrix4d forwardKinematics(double th[6]){
    VectorXd thV = Eigen::Map<Eigen::VectorXd>(th, 1, 6);
    Matrix4d A_1 = AH(1, thV);
    Matrix4d A_2  =AH(2, thV);
    Matrix4d A_3 = AH(3, thV);
    Matrix4d A_4 = AH(4, thV);
    Matrix4d A_5 = AH(5, thV);
    Matrix4d A_6 = AH(6, thV);

    Matrix4d T_6 = A_1 * A_2 * A_3 * A_4 * A_5 * A_6;

    return T_6;
}

/********************** INVERSE KINEMATICS **********************/

/**
 * @brief Compute the inverse kinematics of the robotic arm, allowing retrieval of joint positions.
 *
 * @param desired_position The desired gripper position and orientation.
 * @return A matrix containing multiple solutions for joint angles to achieve the desired position.
 */
Matrix<double, 6, 8> inverseKinematics(Matrix4d& desired_position){
    Matrix<double, 6, 8> thTmp;
    thTmp.setZero();

    Vector4d P_05 = desired_position * Vector4d(0, 0, -d[5], 1) - Vector4d(0, 0, 0, 1);

    double psi = atan2(P_05(1), P_05(0));
    double phi = acos(d[3] / sqrt(P_05(1) * P_05(1) + P_05(0) * P_05(0)));

    // The two solutions for theta1 correspond to the shoulder
    // being either left or right

    thTmp.block<1,4>(0,0).setConstant( M_PI / 2 + psi + phi);
    thTmp.block<1,4>(0,4).setConstant(M_PI / 2 + psi - phi);

    /**** theta5 ****/

    int cl5[2] = {0, 4}; //wrist up or down
    
    for(int i=0; i<2; i++){
        VectorXd subV = thTmp.col(cl5[i]);
        Matrix4d T_10 = AH(1, subV).inverse();
        //Matrix4d T_10 = AH(1, thTmp.block<1,6>(0,cl[i])).inverse();
        Matrix4d T_16 = T_10 * desired_position;
        thTmp.block<1,2>(4,cl5[i]).setConstant(+acos((T_16(2, 3) - d[3]) / d[5]));
        thTmp.block<1,2>(4,cl5[i]+2).setConstant(-acos((T_16(2, 3) - d[3]) / d[5]));
    }

    // **** theta6 ****
    // theta6 is not well-defined when sin(theta5) = 0 or when T16(1,3), T16(2,3) = 0.

    int cl6[4] = {0, 2, 4, 6};
    for(int i=0; i<4; i++){
        VectorXd subV = thTmp.col(cl6[i]);
        Matrix4d T_10 = AH(1, subV).inverse();
        Matrix4d T_16 = (T_10 * desired_position).inverse();
        thTmp.block<1,2>(5,cl6[i]).setConstant(atan2(-T_16(1, 2) / sin(thTmp(4, cl6[i])), T_16(0, 2) / sin(thTmp(4, cl6[i]))));
    }

    // **** theta3 ****                            

    int cl3[4] = {0, 2, 4, 6};
    for(int i=0; i<4; i++){
        VectorXd subV = thTmp.col(cl3[i]);
        Matrix4d T_10 = AH(1, subV).inverse();
        Matrix4d T_65 = AH(6, subV);
        Matrix4d T_54 = AH(5, subV);
        Matrix4d T_14 = T_10 * desired_position * (T_54 * T_65).inverse();
        Vector4d P_13 = T_14 * Vector4d(0, -d[3], 0, 1) - Vector4d(0, 0, 0, 1);
        double t3 = acos(((P_13).norm()*(P_13).norm() - a[1]*a[1] - a[2]*a[2]) / (2 * a[1] * a[2]));
        //thTmp.row(2).segment(cl[i], cl[i]) = t3;
        thTmp(2,cl3[i]) = t3;
        //thTmp.row(2).segment(cl[i]+1, cl[i]+1) = -t3;
        thTmp(2,cl3[i]+1) = -t3;
    }
    // **** theta2 and theta 4 ****

    int cl24[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    for(int i=0; i<8; i++){
        VectorXd subV = thTmp.col(cl24[i]);
        Matrix4d T_10 = AH(1, subV).inverse();
        Matrix4d T_65 = AH(6, subV).inverse();
        Matrix4d T_54 = AH(5, subV).inverse();
        Matrix4d T_14 = T_10 * desired_position * T_65 * T_54;
        Vector4d P_13 = T_14 * Vector4d(0, -d[3], 0, 1) - Vector4d(0, 0, 0, 1);

    // theta 2

        thTmp(1, cl24[i]) = -atan2(P_13(1), -P_13(0)) + asin(a[2] * sin(thTmp(2, cl24[i])) / (P_13).norm());

    // theta 4

        Matrix4d T_32 = AH(3, subV).inverse();
        Matrix4d T_21 = AH(2, subV).inverse();
        Matrix4d T_34 = T_32 * T_21 * T_14;
        thTmp(3, cl24[i]) = atan2(T_34(1, 0), T_34(0, 0));
    }

    return thTmp;
}

/**
 * @brief Function to retrieve one of the possible solutions to reach the desired gripper position.
 *
 * @param x X-coordinate of the desired position.
 * @param y Y-coordinate of the desired position.
 * @param z Z-coordinate of the desired position.
 * @param rot Rotation matrix representing the desired orientation.
 * @return A vector of joint angles representing a solution to achieve the desired position.
 */
Matrix<double, 6, 1> getJoints(double x, double y, double z, Matrix3d rot){
    // base offset
    z -= 0.771347; //-0.016300

    // create trasform matrix
    Matrix<double, 4, 4> pose;
    pose.col(3) = Vector4d(x, y, z, 1);
    
    pose.block<3,3>(0,0) = rot;
    Matrix<double, 6, 8> thRes = inverseKinematics(pose);
    Matrix<double, 6, 8> result;
    Matrix<double, 6, 8> mPi; 
    Matrix<double, 6, 8> mPi2;
    mPi.block<6,8>(0,0).setConstant(M_PI);
    mPi2.block<6,8>(0,0).setConstant(2*M_PI);
    Eigen::Array<double, 6, 8> thResArray = thRes.array();
    Eigen::Array<double, 6, 8> mPi2Array = mPi2.array();

    // normalize    
    for(int i=0;i<thRes.rows(); i++){
        for(int j=0;j<thRes.cols(); j++){
            result(i,j) = std::fmod((thRes(i,j)+ M_PI),(2 * M_PI)) - M_PI;
        }
    }

    // return 5th kinematic solution (one of the possible solutions)
    return result.col(5);
}

/**
 * @brief Function to retrieve the gripper position and orientation given joint angles.
 *
 * @param joints A vector containing the joint angles.
 * @return An instance of the returnValues struct containing the gripper's position and orientation.
 */
returnValues getPose(std::vector<double> joints){
    double jointS[6];
    std::copy(joints.begin(), joints.end(), jointS); //copy of the whole joints vector into jointS array
    Matrix4d th = forwardKinematics(jointS);
    returnValues retV;
    retV.xYZ.push_back(th(0,3));
    retV.xYZ.push_back(th(1,3));
    double retVz = th(2,3);
    retV.rot = th.block<3,3>(0,0);

    retVz += 0.771347;
    retV.xYZ.push_back(retVz);
    return retV;
}
