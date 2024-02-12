#include <cmath>
#include <complex>
#include "inverseKinematics.h"
#include <iostream>
#include "EigenLib.h"
#include "typedefs.h"

/**
 * @brief Transformation matrix for joint 1 to joint 0.
 * @param th Joint angle for joint 1.
 * @return Transformation matrix T10.
 */

Eigen::Matrix4d T10f(double th){
    Eigen::Matrix4d T10{
                        {cos(th),  -sin(th),  0, 0}, 
                        {sin(th),  cos(th), 0, 0},
                        {0, 0, 1, D(0)}, 
                        {0, 0, 0, 1}
                        };
    return T10;
}     

/**
 * @brief Transformation matrix for joint 2 to joint 1.
 * @param th Joint angle for joint 2.
 * @return Transformation matrix T21.
 */

Eigen::Matrix4d T21f(double th){
    Eigen::Matrix4d T21{
                        {cos(th),  -sin(th),  0, 0}, 
                        {0, 0, -1, 0}, 
                        {sin(th),  cos(th), 0, 0},
                        {0, 0, 0, 1}
                        }; 
    return T21;
}   

/**
 * @brief Transformation matrix for joint 3 to joint 2.
 * @param th Joint angle for joint 3.
 * @return Transformation matrix T32.
 */

Eigen::Matrix4d T32f(double th){
    Eigen::Matrix4d T32{
                        {cos(th),  -sin(th),  0, A[1]}, 
                        {sin(th),  cos(th), 0, 0},
                        {0, 0, 1, D[2]}, 
                        {0, 0, 0, 1}
                        }; 
    return T32;
} 

/**
 * @brief Transformation matrix for joint 4 to joint 3.
 * @param th Joint angle for joint 4.
 * @return Transformation matrix T43.
 */

Eigen::Matrix4d T43f(double th){
    Eigen::Matrix4d T43{
                        {cos(th),  -sin(th),  0, A[2]}, 
                        {sin(th),  cos(th), 0, 0},
                        {0, 0, 1, D[3]}, 
                        {0, 0, 0, 1}
                        }; 
    return T43;
}    

/**
 * @brief Transformation matrix for joint 5 to joint 4.
 * @param th Joint angle for joint 5.
 * @return Transformation matrix T54.
 */

Eigen::Matrix4d T54f(double th){
    Eigen::Matrix4d T54{
                        {cos(th),  -sin(th),  0, 0}, 
                        {0, 0, -1, -D[4]}, 
                        {sin(th),  cos(th), 0, 0},
                        {0, 0, 0, 1}
                        };
    return T54;
}
    
/**
 * @brief Transformation matrix for end effector to joint 5.
 * @param th Joint angle for joint 6.
 * @return Transformation matrix T65.
 */

 Eigen::Matrix4d T65f(double th){
    Eigen::Matrix4d T65{
                        {cos(th),  -sin(th),  0, 0}, 
                        {0, 0, 1, D[5]}, 
                        {-sin(th),  -cos(th), 0, 0},
                        {0, 0, 0, 1}
                        };
    return T65;
}               

/* Matrix86d ur5inverseKinematics(Eigen::Vector3d p60, Eigen::Matrix3d R60){
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

    double th1_1 = std::real( atan2(p50(1),p50(0)) + acos(std::complex<double>(D[3]/hypot(p50(1),p50(0)), 0.0)).real()) + M_PI/2;
    double th1_2 = std::real( atan2(p50(1),p50(0)) - acos(std::complex<double>(D[3]/hypot(p50(1),p50(0)), 0.0)).real()) + M_PI/2;

    // Finding Th5
    double th5_1 = +std::real(acos(std::complex<double>((p60(0)*sin(th1_1) - p60(1)*cos(th1_1)-D[3]), 0.0) / D[5]));
    double th5_2 = -std::real(acos(std::complex<double>((p60(0)*sin(th1_1) - p60(1)*cos(th1_1)-D[3]), 0.0) / D[5]));
    double th5_3 = +std::real(acos(std::complex<double>((p60(0)*sin(th1_2) - p60(1)*cos(th1_2)-D[3]), 0.0) / D[5]));
    double th5_4 = -std::real(acos(std::complex<double>((p60(0)*sin(th1_2) - p60(1)*cos(th1_2)-D[3]), 0.0) / D[5]));

    T60 = T60.inverse().eval();

    Eigen::Vector3d Xhat = T60.block<3,1>(0,0);
    Eigen::Vector3d Yhat = T60.block<3,1>(0,1);

    // Finding Th6
    double th6_1 = std::real(atan2(((-Xhat(1)*sin(th1_1)+Yhat(1)*cos(th1_1)))/sin(th5_1), ((Xhat(0)*sin(th1_1)-Yhat(0)*cos(th1_1)))/sin(th5_1)));
    double th6_2 = std::real(atan2(((-Xhat(1)*sin(th1_1)+Yhat(1)*cos(th1_1)))/sin(th5_2), ((Xhat(0)*sin(th1_1)-Yhat(0)*cos(th1_1)))/sin(th5_2)));
    double th6_3 = std::real(atan2(((-Xhat(1)*sin(th1_2)+Yhat(1)*cos(th1_2)))/sin(th5_3), ((Xhat(0)*sin(th1_2)-Yhat(0)*cos(th1_2)))/sin(th5_3)));
    double th6_4 = std::real(atan2(((-Xhat(1)*sin(th1_2)+Yhat(1)*cos(th1_2)))/sin(th5_4), ((Xhat(0)*sin(th1_2)-Yhat(0)*cos(th1_2)))/sin(th5_4)));
  
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

    double th3_1 = acos( std::complex<double>((pow(p41xz_1,2)-pow(A[1],2)-pow(A[2],2)) / (2*A[1]*A[2]), 0.0)).real();
    double th3_2 = acos( std::complex<double>((pow(p41xz_2,2)-pow(A[1],2)-pow(A[2],2)) / (2*A[1]*A[2]), 0.0)).real();
    double th3_3 = acos( std::complex<double>((pow(p41xz_3,2)-pow(A[1],2)-pow(A[2],2)) / (2*A[1]*A[2]), 0.0)).real();
    double th3_4 = acos( std::complex<double>((pow(p41xz_4,2)-pow(A[1],2)-pow(A[2],2)) / (2*A[1]*A[2]), 0.0)).real();

    double th3_5 = -th3_1;
    double th3_6 = -th3_2;
    double th3_7 = -th3_3;
    double th3_8 = -th3_4;

    double th2_1 = std::real( atan2(-p41_1(2), -p41_1(0))-asin((-A[3]*sin(th3_1))/p41xz_1));
    double th2_2 = std::real( atan2(-p41_2(2), -p41_2(0))-asin((-A[3]*sin(th3_2))/p41xz_2));
    double th2_3 = std::real( atan2(-p41_3(2), -p41_3(0))-asin((-A[3]*sin(th3_3))/p41xz_3));
    double th2_4 = std::real( atan2(-p41_4(2), -p41_4(0))-asin((-A[3]*sin(th3_4))/p41xz_4));
    
    double th2_5 = std::real( atan2(-p41_1(2), -p41_1(0))-asin((A[3]*sin(th3_1))/p41xz_1));
    double th2_6 = std::real( atan2(-p41_2(2), -p41_2(0))-asin((A[3]*sin(th3_2))/p41xz_2));
    double th2_7 = std::real( atan2(-p41_3(2), -p41_3(0))-asin((A[3]*sin(th3_3))/p41xz_3));
    double th2_8 = std::real( atan2(-p41_4(2), -p41_4(0))-asin((A[3]*sin(th3_4))/p41xz_4));

    Eigen::Matrix4d T43 = ((((T32f(th3_1)).inverse()*(T21f(th2_1)).inverse())*((T10f(th1_1)).inverse()*T60))*((T65f(th6_1)).inverse()*(T54f(th5_1)).inverse()));
    Eigen::Vector3d Xhat43 = T43.block<3,1>(0,3);
    double th4_1 = std::real(atan2(Xhat43(1),Xhat43(0)));

    T43 = ((((T32f(th3_2)).inverse()*(T21f(th2_2)).inverse())*((T10f(th1_1)).inverse()*T60))*((T65f(th6_2)).inverse()*(T54f(th5_2)).inverse()));
    Xhat43 = T43.block<3,1>(0,0);
    double th4_2 = std::real(atan2(Xhat43(1),Xhat43(0)));

    T43 = ((((T32f(th3_3)).inverse()*(T21f(th2_3)).inverse())*((T10f(th1_2)).inverse()*T60))*((T65f(th6_3)).inverse()*(T54f(th5_3)).inverse()));
    Xhat43 = T43.block<3,1>(0,0);
    double th4_3 = std::real(atan2(Xhat43(1),Xhat43(0)));

    T43 = ((((T32f(th3_4)).inverse()*(T21f(th2_4)).inverse())*((T10f(th1_2)).inverse()*T60))*((T65f(th6_4)).inverse()*(T54f(th5_4)).inverse()));
    Xhat43 = T43.block<3,1>(0,0);
    double th4_4 = std::real(atan2(Xhat43(1),Xhat43(0)));
    
    T43 = ((((T32f(th3_5)).inverse()*(T21f(th2_5)).inverse())*((T10f(th1_1)).inverse()*T60))*((T65f(th6_1)).inverse()*(T54f(th5_1)).inverse()));
    Xhat43 = T43.block<3,1>(0,0);
    double th4_5 = std::real(atan2(Xhat43(1),Xhat43(0)));

    T43 = ((((T32f(th3_6)).inverse()*(T21f(th2_6)).inverse())*((T10f(th1_1)).inverse()*T60))*((T65f(th6_2)).inverse()*(T54f(th5_2)).inverse()));
    Xhat43 = T43.block<3,1>(0,0);
    double th4_6 = std::real(atan2(Xhat43(1),Xhat43(0)));

    T43 = ((((T32f(th3_7)).inverse()*(T21f(th2_7)).inverse())*((T10f(th1_2)).inverse()*T60))*((T65f(th6_3)).inverse()*(T54f(th5_3)).inverse()));
    Xhat43 = T43.block<3,1>(0,0);
    double th4_7 = std::real(atan2(Xhat43(1),Xhat43(0)));

    T43 = ((((T32f(th3_8)).inverse()*(T21f(th2_8)).inverse())*((T10f(th1_2)).inverse()*T60))*((T65f(th6_4)).inverse()*(T54f(th5_4)).inverse()));
    Xhat43 = T43.block<3,1>(0,0);
    double th4_8 = std::real(atan2(Xhat43(1),Xhat43(0)));

    Matrix86d Th{
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
} */


/**
 * @brief Check if a given value is almost zero.
 * @param x Value to check.
 * @return True if the value is almost zero, false otherwise.
 */

bool almostZero(double x){
    return (abs(x)< 1e-7);
}

/**
 * @brief General transformation matrix for a specific joint and angle.
 * @param th Joint angle.
 * @param i Joint index.
 * @return Transformation matrix.
 */

Eigen::Matrix4d generalTransformationMatrix(double th,int i){
    return Eigen::Matrix4d{
    {cos(th), -sin(th)*cos(Alpha(i)), sin(th)*sin(Alpha(i)), A(i)*cos(th)},
    {sin(th), cos(th)*cos(Alpha(i)), -cos(th)*sin(Alpha(i)), A(i)*sin(th)},
    {0, sin(Alpha(i)), cos(Alpha(i)), D(i)},
    {0, 0, 0, 1}
    };
}

/**
 * @brief Inverse kinematics for the UR5 robot arm with error handling.
 * @param p60 End effector position.
 * @param R60 End effector orientation matrix.
 * @return Matrix of joint angles corresponding to multiple solutions.
 */

Matrix68d ur5inverseKinematics(Eigen::Vector3d p60, Eigen::Matrix3d R60){
    Eigen::Matrix4d T60 {
                        {R60(0,0),  R60(0,1),  R60(0,2), p60(0)}, 
                        {R60(1,0),  R60(1,1),  R60(1,2), p60(1)},
                        {R60(2,0),  R60(2,1),  R60(2,2), p60(2)}, 
                        {0,         0,         0,        1}
                        };

    // Finding Th1
    Eigen::Vector4d x {0, 0, -D(5), 1};
    //x = x.transpose(); 
    Eigen::Vector4d p50 = T60*x;

    double psi = atan2(p50(1), p50(0));
    double p50xy = hypot(p50(1), p50(0));

    if(p50xy < D(3)){
        //th = nan*Vector6d{1,1,1,1,1,1};
        std::cout << "WARNING: Position request in the unreachable cylinder" << std::endl;
        Matrix86d::Zero();
    }
    double phi1_1 = acos(D(3)/p50xy);
    double phi1_2 = -phi1_1;

    double th1_1 = psi+phi1_1+M_PI/2;
    double th1_2 = psi+phi1_2+M_PI/2;

    double p61z_1 = p60(0)*sin(th1_1)-p60(1)*cos(th1_1);
    double p61z_2 = p60(0)*sin(th1_2)-p60(1)*cos(th1_2);

    double th5_1_1 = acos((p61z_1-D(3))/D(5));
    double th5_1_2 = -acos((p61z_1-D(3))/D(5));
    double th5_2_1 = acos((p61z_2-D(3))/D(5));
    double th5_2_2 = -acos((p61z_2-D(3))/D(5));

    Eigen::Matrix4d T10_1 = generalTransformationMatrix(th1_1,0);
    Eigen::Matrix4d T10_2 = generalTransformationMatrix(th1_2,0);
    
    Eigen::Matrix4d T16_1 = ((T10_1).inverse()*T60).inverse();
    Eigen::Matrix4d T16_2 = ((T10_2).inverse()*T60).inverse();
    
    double zy_1 = T16_1(1,2);
    double zx_1 = T16_1(0,2);

    double zy_2 = T16_2(1,2);
    double zx_2 = T16_2(0,2);

    double th6_1_1, th6_1_2, th6_2_1, th6_2_2;

    if(almostZero(sin(th5_1_1)) || (almostZero(zy_1) && almostZero(zx_1))){
        std::cout << "Singular configuration. Choosing arbitrary th6" << std::endl;
        th6_1_1 = 0;
    }else{
        th6_1_1 = atan2((-zy_1/sin(th5_1_1)), (zx_1/sin(th5_1_1)));
    }
    
    if(almostZero(sin(th5_1_2)) || (almostZero(zy_1) && almostZero(zx_1))){
        std::cout << "Singular configuration. Choosing arbitrary th6" << std::endl;
        th6_1_2 = 0;
    }else{
        th6_1_2 = atan2((-zy_1/sin(th5_1_2)), (zx_1/sin(th5_1_2)));
    }

    if(almostZero(sin(th5_2_1)) || (almostZero(zy_2) && almostZero(zx_2))){
        std::cout << "Singular configuration. Choosing arbitrary th6" << std::endl;
        th6_2_1 = 0;
    }else{
        th6_2_1 = atan2((-zy_2/sin(th5_2_1)), (zx_2/sin(th5_2_1)));
    }

    if(almostZero(sin(th5_2_2)) || (almostZero(zy_2) && almostZero(zx_2))){
        std::cout << "Singular configuration. Choosing arbitrary th6" << std::endl;
        th6_2_2 = 0;
    }else{
        th6_2_2 = atan2((-zy_2/sin(th5_2_2)), (zx_2/sin(th5_2_2)));
    }

    Eigen::Matrix4d T61_1 = (T16_1).inverse();
    Eigen::Matrix4d T61_2 = (T16_2).inverse();

    Eigen::Matrix4d T54_1_1 = generalTransformationMatrix(th5_1_1, 4);
    Eigen::Matrix4d T54_1_2 = generalTransformationMatrix(th5_1_2, 4);
    Eigen::Matrix4d T54_2_1 = generalTransformationMatrix(th5_2_1, 4);
    Eigen::Matrix4d T54_2_2 = generalTransformationMatrix(th5_2_2, 4);

    Eigen::Matrix4d T65_1_1 = generalTransformationMatrix(th6_1_1, 5);
    Eigen::Matrix4d T65_1_2 = generalTransformationMatrix(th6_1_2, 5);
    Eigen::Matrix4d T65_2_1 = generalTransformationMatrix(th6_2_1, 5);
    Eigen::Matrix4d T65_2_2 = generalTransformationMatrix(th6_2_2, 5);

    Eigen::Matrix4d T41_1_1 = T61_1*(T54_1_1*T65_1_1).inverse();
    Eigen::Matrix4d T41_1_2 = T61_1*(T54_1_2*T65_1_2).inverse();
    Eigen::Matrix4d T41_2_1 = T61_2*(T54_2_1*T65_2_1).inverse();
    Eigen::Matrix4d T41_2_2 = T61_2*(T54_2_2*T65_2_2).inverse();
    
    Eigen::Vector4d P = T41_1_1*(Eigen::Vector4d{0,-D(3),0,1}); 
    Eigen::Vector3d P31_1_1 = P.block<3,1>(0,0);
    P = T41_1_2*(Eigen::Vector4d{0,-D(3),0,1}); 
    Eigen::Vector3d P31_1_2 = P.block<3,1>(0,0);
    P = T41_2_1*(Eigen::Vector4d{0,-D(3),0,1}); 
    Eigen::Vector3d P31_2_1 = P.block<3,1>(0,0);
    P = T41_2_2*(Eigen::Vector4d{0,-D(3),0,1}); 
    Eigen::Vector3d P31_2_2 = P.block<3,1>(0,0);

    double th3_1_1_1, th3_1_1_2, th3_1_2_1, th3_1_2_2, th3_2_1_1, th3_2_1_2, th3_2_2_1, th3_2_2_2;
    double C = (pow((P31_1_1).norm(),2)-A(1)*A(1)-A(2)*A(2))/(2*A(1)*A(2));
    if (abs(C) > 1){
        std::cout << "Point out of the work space" << std::endl;
        //th3_1_1_1 = NULL;
        //th3_1_1_2 = NULL;
    }else{
        th3_1_1_1 = acos(C);
        th3_1_1_2 = -acos(C);
    }
    C = (pow((P31_1_2).norm(),2)-A(1)*A(1)-A(2)*A(2))/(2*A(1)*A(2));
    if (abs(C) > 1){
        std::cout << "Point out of the work space" << std::endl;
        //th3_1_2_1 = NULL;
        //th3_1_2_2 = NULL;
    }else{
        th3_1_2_1 = acos(C);
        th3_1_2_2 = -acos(C);
    }
    C = (pow((P31_2_1).norm(),2)-A(1)*A(1)-A(2)*A(2))/(2*A(1)*A(2));
    if (abs(C) > 1){
        std::cout << "Point out of the work space" << std::endl;
        //th3_2_1_1 = NULL;
        //th3_2_1_2 = NULL;
    }else{
        th3_2_1_1 = acos(C);
        th3_2_1_2 = -acos(C);
    }
    C = (pow((P31_2_2).norm(),2)-A(1)*A(1)-A(2)*A(2))/(2*A(1)*A(2));
    if (abs(C) > 1){
        std::cout << "Point out of the work space" << std::endl;
        //th3_2_2_1 = NULL;
        //th3_2_2_2 = NULL;
    }else{
        th3_2_2_1 = acos(C);
        th3_2_2_2 = -acos(C);
    }

    double th2_1_1_1 = -atan2(P31_1_1(1), -P31_1_1(0))+asin((A(2)*sin(th3_1_1_1))/(P31_1_1).norm());
    double th2_1_1_2 = -atan2(P31_1_1(1), -P31_1_1(0))+asin((A(2)*sin(th3_1_1_2))/(P31_1_1).norm());
    double th2_1_2_1 = -atan2(P31_1_2(1), -P31_1_2(0))+asin((A(2)*sin(th3_1_2_1))/(P31_1_2).norm());
    double th2_1_2_2 = -atan2(P31_1_2(1), -P31_1_2(0))+asin((A(2)*sin(th3_1_2_2))/(P31_1_2).norm());
    double th2_2_1_1 = -atan2(P31_2_1(1), -P31_2_1(0))+asin((A(2)*sin(th3_2_1_1))/(P31_2_1).norm());
    double th2_2_1_2 = -atan2(P31_2_1(1), -P31_2_1(0))+asin((A(2)*sin(th3_2_1_2))/(P31_2_1).norm());
    double th2_2_2_1 = -atan2(P31_2_2(1), -P31_2_2(0))+asin((A(2)*sin(th3_2_2_1))/(P31_2_2).norm());
    double th2_2_2_2 = -atan2(P31_2_2(1), -P31_2_2(0))+asin((A(2)*sin(th3_2_2_2))/(P31_2_2).norm());

    Eigen::Matrix4d T21 = generalTransformationMatrix(th2_1_1_1, 1);
    Eigen::Matrix4d T32 = generalTransformationMatrix(th3_1_1_1, 2);
    Eigen::Matrix4d T41 = T41_1_1;
    Eigen::Matrix4d T43 = (T21*T32).inverse()*T41;
    double xy = T43(2,1);
    double xx = T43(1,1);
    double th4_1_1_1 = atan2(xy,xx);

    T21 = generalTransformationMatrix(th2_1_1_2, 1);
    T32 = generalTransformationMatrix(th3_1_1_2, 2);
    T41 = T41_1_1;
    T43 = (T21*T32).inverse()*T41;
    xy = T43(2,1);
    xx = T43(1,1);
    double th4_1_1_2 = atan2(xy,xx);

    T21 = generalTransformationMatrix(th2_1_2_1, 1);
    T32 = generalTransformationMatrix(th3_1_2_1, 2);
    T41 = T41_1_2;
    T43 = (T21*T32).inverse()*T41;
    xy = T43(2,1);
    xx = T43(1,1);
    double th4_1_2_1 = atan2(xy,xx);

    T21 = generalTransformationMatrix(th2_1_2_2, 1);
    T32 = generalTransformationMatrix(th3_1_2_2, 2);
    T41 = T41_1_2;
    T43 = (T21*T32).inverse()*T41;
    xy = T43(2,1);
    xx = T43(1,1);
    double th4_1_2_2 = atan2(xy,xx);

    T21 = generalTransformationMatrix(th2_2_1_1, 1);
    T32 = generalTransformationMatrix(th3_2_1_1, 2);
    T41 = T41_2_1;
    T43 = (T21*T32).inverse()*T41;
    xy = T43(2,1);
    xx = T43(1,1);
    double th4_2_1_1 = atan2(xy,xx);

    T21 = generalTransformationMatrix(th2_2_1_2, 1);
    T32 = generalTransformationMatrix(th3_2_1_2, 2);
    T41 = T41_2_1;
    T43 = (T21*T32).inverse()*T41;
    xy = T43(2,1);
    xx = T43(1,1);
    double th4_2_1_2 = atan2(xy,xx);

    T21 = generalTransformationMatrix(th2_2_2_1, 1);
    T32 = generalTransformationMatrix(th3_2_2_1, 2);
    T41 = T41_2_2;
    T43 = (T21*T32).inverse()*T41;
    xy = T43(2,1);
    xx = T43(1,1);
    double th4_2_2_1 = atan2(xy,xx);

    T21 = generalTransformationMatrix(th2_2_2_2, 1);
    T32 = generalTransformationMatrix(th3_2_2_2, 2);
    T41 = T41_2_2;
    T43 = (T21*T32).inverse()*T41;
    xy = T43(2,1);
    xx = T43(1,1);
    double th4_2_2_2 = atan2(xy,xx);

    Matrix68d Th {
        {th1_1      ,th1_1       ,th1_1       ,th1_1       ,th1_2      ,th1_2       ,th1_2       ,th1_2},
        {th2_1_1_1  ,th2_1_1_2   ,th2_1_2_1   ,th2_1_2_2   ,th2_2_2_1  ,th2_2_1_2   ,th2_2_2_1   ,th2_2_2_2},
        {th3_1_1_1  ,th3_1_1_2   ,th3_1_2_1   ,th3_1_2_2   ,th3_2_2_1  ,th3_2_1_2   ,th3_2_2_1   ,th3_2_2_2},
        {th4_1_1_1  ,th4_1_1_2   ,th4_1_2_1   ,th4_1_2_2   ,th4_2_2_1  ,th4_2_1_2   ,th4_2_2_1   ,th4_2_2_2},
        {th5_1_1    ,th5_1_1     ,th5_1_2     ,th5_1_2     ,th5_2_2    ,th5_2_1     ,th5_2_2     ,th5_2_2},
        {th6_1_1    ,th6_1_1     ,th6_1_2     ,th6_1_2     ,th6_2_2    ,th6_2_1     ,th6_2_2     ,th6_2_2}
    };
    return Th;
}