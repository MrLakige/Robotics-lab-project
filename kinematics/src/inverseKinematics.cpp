#include <cmath>
#include <complex>
#include "../include/inverseKinematics.h"

Eigen::Matrix<double,6,1> A = {0, -0.425, -0.3922, 0, 0, 0};
Eigen::Matrix<double,6,1> D = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};

Eigen::Matrix4d T10f(double th){
    Eigen::Matrix4d T10{
                        {cos(th),  -sin(th),  0, 0}, 
                        {sin(th),  cos(th), 0, 0},
                        {0, 0, 1, D(0)}, 
                        {0, 0, 0, 1}
                        };
    return T10;
}     

Eigen::Matrix4d T21f(double th){
    Eigen::Matrix4d T21{
                        {cos(th),  -sin(th),  0, 0}, 
                        {0, 0, -1, 0}, 
                        {sin(th),  cos(th), 0, 0},
                        {0, 0, 0, 1}
                        }; 
    return T21;
}   

Eigen::Matrix4d T32f(double th){
    Eigen::Matrix4d T32{
                        {cos(th),  -sin(th),  0, A[1]}, 
                        {sin(th),  cos(th), 0, 0},
                        {0, 0, 1, D[2]}, 
                        {0, 0, 0, 1}
                        }; 
    return T32;
} 

Eigen::Matrix4d T43f(double th){
    Eigen::Matrix4d T43{
                        {cos(th),  -sin(th),  0, A[2]}, 
                        {sin(th),  cos(th), 0, 0},
                        {0, 0, 1, D[3]}, 
                        {0, 0, 0, 1}
                        }; 
    return T43;
}    

Eigen::Matrix4d T54f(double th){
    Eigen::Matrix4d T54{
                        {cos(th),  -sin(th),  0, 0}, 
                        {0, 0, -1, -D[4]}, 
                        {sin(th),  cos(th), 0, 0},
                        {0, 0, 0, 1}
                        };
    return T54;
}
    
 Eigen::Matrix4d T65f(double th){
    Eigen::Matrix4d T65{
                        {cos(th),  -sin(th),  0, 0}, 
                        {0, 0, 1, D[5]}, 
                        {-sin(th),  -cos(th), 0, 0},
                        {0, 0, 0, 1}
                        };
    return T65;
}               

Eigen::Matrix<double, 6, 8> ur5inverseKinematics(Eigen::Vector3d p60, Eigen::Matrix3d R60){

    Eigen::Matrix4d T60 {
                        {R60(0,0),  R60(0,1),  R60(0,2), p60(0)}, 
                        {R60(1,0),  R60(1,1),  R60(1,2), p60(1)},
                        {R60(2,0),  R60(2,1),  R60(2,2), p60(2)}, 
                        {0,         0,         0,        1}
                        };

    // Finding Th1
    Eigen::Vector4d x {0, 0, -D(5), 1};
    x = x.transpose();
    Eigen::Vector4d p50 = T60*x;

    double th1_1 = std::real(atan2(p50(1), p50(0)) + acos(D(3) / hypot(p50(1), p50(0)))) + (M_PI/2);
    double th1_2 = std::real(atan2(p50(1), p50(0)) - acos(D(3) / hypot(p50(1), p50(0)))) + (M_PI/2);

    // Finding Th5
    double th5_1 = +std::real(acos(p60(0)*sin(th1_1)) - p60(1)*cos(th1_1)-D(3) / D(5));
    double th5_2 = -std::real(acos(p60(0)*sin(th1_1)) - p60(1)*cos(th1_1)-D(3) / D(5));
    double th5_3 = +std::real(acos(p60(0)*sin(th1_2)) - p60(1)*cos(th1_2)-D(3) / D(5));
    double th5_4 = -std::real(acos(p60(0)*sin(th1_2)) - p60(1)*cos(th1_2)-D(3) / D(5));

    T60 = T60.inverse();

    Eigen::Vector3d Xhat = T60.block<3,1>(0,0);
    Eigen::Vector3d Yhat = T60.block<3,1>(0,1);

    double th6_1 = std::real(atan2(((-Xhat[1]*sin(th1_1)+Yhat[1]*cos(th1_1)))/sin(th5_1), ((Xhat[0]*sin(th1_1)-Yhat[0]*cos(th1_1)))/sin(th5_1)));
    double th6_2 = std::real(atan2(((-Xhat[1]*sin(th1_1)+Yhat[1]*cos(th1_1)))/sin(th5_2), ((Xhat[0]*sin(th1_1)-Yhat[0]*cos(th1_1)))/sin(th5_2)));
    double th6_3 = std::real(atan2(((-Xhat[1]*sin(th1_2)+Yhat[1]*cos(th1_2)))/sin(th5_3), ((Xhat[0]*sin(th1_2)-Yhat[0]*cos(th1_2)))/sin(th5_3)));
    double th6_4 = std::real(atan2(((-Xhat[1]*sin(th1_2)+Yhat[1]*cos(th1_2)))/sin(th5_4), ((Xhat[0]*sin(th1_2)-Yhat[0]*cos(th1_2)))/sin(th5_4)));

    //---- DA RIGA 66 ----    
    Eigen::Matrix4d T41 = ((T10f(th1_1).inverse())*T60)*((T65f(th6_1).inverse())*(T54f(th5_1).inverse()));
    Eigen::Vector3d p41_1 = T41.block<3,1>(0,3);
    double p41xz_1 = hypot(p41_1(0), p41_1(2));

    T41 = ((T10f(th1_1).inverse())*T60)*((T65f(th6_2).inverse())*(T54f(th5_2).inverse()));
    Eigen::Vector3d p41_2 = T41.block<3,1>(0,3);
    double p41xz_2 = hypot(p41_2(0), p41_2(2));

    T41 = ((T10f(th1_2).inverse())*T60)*((T65f(th6_3).inverse())*(T54f(th5_3).inverse()));
    Eigen::Vector3d p41_3 = T41.block<3,1>(0,3);
    double p41xz_3 = hypot(p41_3(0), p41_3(2));

    T41 = ((T10f(th1_2).inverse())*T60)*((T65f(th6_4).inverse())*(T54f(th5_4).inverse()));
    Eigen::Vector3d p41_4 = T41.block<3,1>(0,3);
    double p41xz_4 = hypot(p41_4(0), p41_4(2));

    double th3_1 = std::real(acos((pow(p41xz_1,2)-pow(A[1],2)-pow(A[2],2))/(2*A[1]*A[2])));
    double th3_2 = std::real(acos((pow(p41xz_2,2)-pow(A[1],2)-pow(A[2],2))/(2*A[1]*A[2])));
    double th3_3 = std::real(acos((pow(p41xz_3,2)-pow(A[1],2)-pow(A[2],2))/(2*A[1]*A[2])));
    double th3_4 = std::real(acos((pow(p41xz_4,2)-pow(A[1],2)-pow(A[2],2))/(2*A[1]*A[2])));

    double th3_5 = -th3_1;
    double th3_6 = -th3_2;
    double th3_7 = -th3_3;
    double th3_8 = -th3_4;

    double th2_1 = std::real(atan2(-p41_1[2], -p41_1[0])-asin((-A[2]*sin(th3_1))/p41xz_1));
    double th2_2 = std::real(atan2(-p41_2[2], -p41_2[0])-asin((-A[2]*sin(th3_2))/p41xz_2));
    double th2_3 = std::real(atan2(-p41_3[2], -p41_3[0])-asin((-A[2]*sin(th3_3))/p41xz_3));
    double th2_4 = std::real(atan2(-p41_4[2], -p41_4[0])-asin((-A[2]*sin(th3_4))/p41xz_4));
    
    double th2_5 = std::real(atan2(-p41_1[2], -p41_1[0])-asin((A[2]*sin(th3_1))/p41xz_1));
    double th2_6 = std::real(atan2(-p41_2[2], -p41_2[0])-asin((A[2]*sin(th3_2))/p41xz_2));
    double th2_7 = std::real(atan2(-p41_3[2], -p41_3[0])-asin((A[2]*sin(th3_3))/p41xz_3));
    double th2_8 = std::real(atan2(-p41_4[2], -p41_4[0])-asin((A[2]*sin(th3_4))/p41xz_4));

    Eigen::Matrix4d T43 = ((((T32f(th3_1)).inverse()*(T21f(th2_1)).inverse())*((T10f(th1_1)).inverse()*T60))*((T65f(th6_1)).inverse()*(T54f(th5_1)).inverse()));
    Eigen::Vector3d Xhat43 = T43.block<3,1>(0,3);
    double th4_1 = std::real(atan2(Xhat43(1),Xhat43(0)));

    T43 = ((((T32f(th3_2)).inverse()*(T21f(th2_2)).inverse())*((T10f(th1_1)).inverse()*T60))*((T65f(th6_2)).inverse()*(T54f(th5_2)).inverse()));
    Xhat43 = T43.block<3,1>(0,3);
    double th4_2 = std::real(atan2(Xhat43(1),Xhat43(0)));

    T43 = ((((T32f(th3_3)).inverse()*(T21f(th2_3)).inverse())*((T10f(th1_2)).inverse()*T60))*((T65f(th6_3)).inverse()*(T54f(th5_3)).inverse()));
    Xhat43 = T43.block<3,1>(0,3);
    double th4_3 = std::real(atan2(Xhat43(1),Xhat43(0)));

    T43 = ((((T32f(th3_4)).inverse()*(T21f(th2_4)).inverse())*((T10f(th1_2)).inverse()*T60))*((T65f(th6_4)).inverse()*(T54f(th5_4)).inverse()));
    Xhat43 = T43.block<3,1>(0,3);
    double th4_4 = std::real(atan2(Xhat43(1),Xhat43(0)));
    
    T43 = ((((T32f(th3_5)).inverse()*(T21f(th2_5)).inverse())*((T10f(th1_1)).inverse()*T60))*((T65f(th6_1)).inverse()*(T54f(th5_1)).inverse()));
    Xhat43 = T43.block<3,1>(0,3);
    double th4_5 = std::real(atan2(Xhat43(1),Xhat43(0)));

    T43 = ((((T32f(th3_6)).inverse()*(T21f(th2_6)).inverse())*((T10f(th1_1)).inverse()*T60))*((T65f(th6_2)).inverse()*(T54f(th5_2)).inverse()));
    Xhat43 = T43.block<3,1>(0,3);
    double th4_6 = std::real(atan2(Xhat43(1),Xhat43(0)));

    T43 = ((((T32f(th3_7)).inverse()*(T21f(th2_7)).inverse())*((T10f(th1_2)).inverse()*T60))*((T65f(th6_3)).inverse()*(T54f(th5_3)).inverse()));
    Xhat43 = T43.block<3,1>(0,3);
    double th4_7 = std::real(atan2(Xhat43(1),Xhat43(0)));

    T43 = ((((T32f(th3_8)).inverse()*(T21f(th2_8)).inverse())*((T10f(th1_2)).inverse()*T60))*((T65f(th6_4)).inverse()*(T54f(th5_4)).inverse()));
    Xhat43 = T43.block<3,1>(0,3);
    double th4_8 = std::real(atan2(Xhat43(1),Xhat43(0)));

    Eigen::Matrix<double, 6, 8> Th {
                                        {th1_1, th2_1, th3_1, th4_1, th5_1, th6_1},
                                        {th1_1, th2_2, th3_2, th4_2, th5_2, th6_2},
                                        {th1_2, th2_3, th3_3, th4_3, th5_3, th6_3},
                                        {th1_2, th2_4, th3_4, th4_4, th5_4, th6_4},
                                        {th1_1, th2_5, th3_5, th4_5, th5_1, th6_1},
                                        {th1_1, th2_6, th3_6, th4_6, th5_2, th6_2},
                                        {th1_2, th2_7, th3_7, th4_7, th5_3, th6_3},
                                        {th1_2, th2_8, th3_8, th4_8, th5_4, th6_4},
                                    };

    return Th;
}