#include <iostream>
#include "Eigen/Dense"
#include <cmath>

using namespace Eigen;

Matrix4d AH(int n, double th[6]){
    Matrix4d T_a = Matrix4d::Identitiy();
    T_a(0, 3) = a[n-1];

    Matrix4d T_d = Matrix4d::Identitiy();
    T_d(2, 3) = d[n-1];

    Matrix4d Rzt << cos(th[n - 1]), -sin(th[n - 1]), 0, 0,
                    sin(th[n - 1]), cos(th[n - 1]),  0, 0,
                    0,              0,               1, 0,
                    0,              0,               0, 1;

    Matrix4d Rzt << 1, 0,                0,                 0,
                    0, cos(alph[n - 1]), -sin(alph[n - 1]), 0,
                    0, sin(alph[n - 1]), cos(alph[n - 1]),  0,
                    0, 0,                0,                 1;

    return T_d * Rzt * T_a * Rxa;
}

Matrix4d forwardKinematics(double th[6]){
    Matrix4d A_1 AH(1, th);
    Matrix4d A_2 AH(2, th);
    Matrix4d A_3 AH(3, th);
    Matrix4d A_4 AH(4, th);
    Matrix4d A_5 AH(5, th);
    Matrix4d A_6 AH(6, th);

    Matrix4d T_6 = A_1 * A_2 * A_3 * A_4 * A_5 * A_6;

    return T_6;
}

Matrix<double, 6, 8> inverseKinematics(Matrix4d& desired_position){
    Matrix<double, 6, 8> thTmp;
    thTmp.setZero();

    Vector4d P_05 = desired_position * Vector4d(0, 0, -d[5], 1) - Vector4d(0, 0, 0, 1);
    double P_05_flattened = {P_05(0), P_05(1), P_05(2), P_05(3)};

    double psi = atan2(P_05(1), P_05(0));
    double phi = acos(d[3] / sqrt(P_05(1) * P_05(1) + P_05(0) * P_05(0)));

    // The two solutions for theta1 correspond to the shoulder
    // being either left or right

    thTmp.row(0).segment(0,4) = M_PI / 2 + psi + phi;
    thTmp.row(0).segment(4,8) = M_PI / 2 + psi - phi;

    /**** theta5 ****/

    int cl[2] = [0, 4];
    
    for(int i=0, i<2; i++){
        Matrix4d T_10 = AH(1, thTmp.col(cl[i])).inverse();
        Matrix4d T_16 = T_10 * desired_position;
        thTmp.row(4).segment(cl[i], cl[i]+2) = +acos((T_16(2, 3) - d[3]) / d[5]);
        thTmp.row(4).segment(cl[i]+2, cl[i]+4) = -acos((T_16(2, 3) - d[3]) / d[5]);
    }

    // **** theta6 ****
    // theta6 is not well-defined when sin(theta5) = 0 or when T16(1,3), T16(2,3) = 0.

    int cl[4] = [0, 2, 4, 6];
    for(int i=0, i<4; i++){
        Matrix4d T_10 = AH(1, thTmp.col(cl[i])).inverse();
        Matrix4d T_16 = (T_10 * desired_position).inverse();
        thTmp.row(5).segment(cl[i], cl[i]+2) = atan2(-T_16(1, 2) / sin(th[4, cl[i]]), T_16[0, 2] / sin(thTmp.row(4).segment(cl[i], cl[i])));
    }

    // **** theta3 ****                            

    int cl[4] = [0, 2, 4, 6];
    for(int i=0, i<4; i++){
        Matrix4d T_10 = AH(1, thTmp.col(cl[i])).inverse();
        Matrix4d T_65 = AH(6, thTmp.col(cl[i]));
        Matrix4d T_54 = AH(5, thTmp.col(cl[i]));
        Matrix4d T_14 = T_10 * desired_position * (T_54 * T_65).inverse();
        Vector4d P_13 = T_14 * Vector4d(0, -d[3], 0, 1) - Vector4d(0, 0, 0, 1);
        double t3 = acos(((P_13).norm*(P_13).norm - a[1]*a[1] - a[2]*a[2]) / (2 * a[1] * a[2])) 
        //thTmp.row(2).segment(cl[i], cl[i]) = t3;
        thTmp(2,cl[i]) = t3;
        //thTmp.row(2).segment(cl[i]+1, cl[i]+1) = -t3;
        thTmp(2,cl[i]+1) = -t3;
    }
    // **** theta2 and theta 4 ****

    int cl[8] = [0, 1, 2, 3, 4, 5, 6, 7]
    for(int i=0, i<8; i++){
        Matrix4d T_10 = AH(1, thTmp.col(cl[i])).inverse();
        Matrix4d T_65 = AH(6, thTmp.col(cl[i])).inverse();
        Matrix4d T_54 = AH(5, thTmp.col(cl[i])).inverse();
        Matrix4d T_14 = T_10 * desired_positon * T_65 * T_54;
        Vector4d P_13 = T_14 * Vector4d(0, -d[3], 0, 1) - Vector4d(0, 0, 0, 1);

    // theta 2

        thTmp.(1, cl[i]) = -atan2(P_13(1), -P_13(0)) + asin(a[2] * sin(thTmp.row(2).segment(cl[i], cl[i])) / (P_13).norm());

    // theta 4

        Matrix4d T_32 = AH(3, thTmp.col(cl[i])).inverse();
        Matrix4d T_21 = AH(2, thTmp.col(cl[i])).inverse();
        Matrix4d T_34 = T_32 * T_21 * T_14;
        thTmp(3, cl[i]) = atan2(T_34(1, 0), T_34(0, 0));
    }

    return thTmp;
}

Matrix<double, 6, 1> getJoints(double x, double y, double z, Matrix3d rot){
    // base offset
    z -= 0.771347 //-0.016300

    // create trasform matrix
    Matrix<double, 4, 4> pose;
    pose.column(3) = Vector4d(x, y, z, 1);
    
    pose.block<3,3>(0,0) = rot;
    Matrix<double, 6, 8> thRes = inverseKinematics(pose);

    // normalize
    thRes = (thRes + M_PI) % (2 * M_PI) - M_PI;

    // return 5th kinematic solution
    return thRes.col(5);
}

returnValues getPose(double joints[6]){
    Matrix4d th = forwardKinematics(joints);

    //x, y, z = th[:3, 3]
    retV->x = th(0,3);
    retV->y = th(1,3);
    retV->z = th(2,3);
    retV->rot = th.block<3,3>(0,0);

    retV->z += 0.771347
    return retV;
}
