#include "../include/directKinematics.h"


directK ur5DirectKinematics(Eigen::Matrix<double,6,1> Th){
    Eigen::Matrix<double,6,1> A = {0, -0.425, -0.3922, 0, 0, 0};
    Eigen::Matrix<double,6,1> D = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};
    
    Eigen::Matrix4d T10{
                        {cos(Th(0)),  -sin(Th(0)),  0, 0}, 
                        {sin(Th(0)),  cos(Th(0)), 0, 0},
                        {0, 0, 1, D(0)}, 
                        {0, 0, 0, 1}
                        };

    Eigen::Matrix4d T21{
                        {cos(Th(1)),  -sin(Th(1)),  0, 0}, 
                        {0, 0, -1, 0}, 
                        {sin(Th(1)),  cos(Th(1)), 0, 0},
                        {0, 0, 0, 1}
                        };    

    Eigen::Matrix4d T32{
                        {cos(Th(2)),  -sin(Th(2)),  0, A[1]}, 
                        {sin(Th(2)),  cos(Th(2)), 0, 0},
                        {0, 0, 1, D[2]}, 
                        {0, 0, 0, 1}
                        }; 

    Eigen::Matrix4d T43{
                        {cos(Th(3)),  -sin(Th(3)),  0, A[2]}, 
                        {sin(Th(3)),  cos(Th(3)), 0, 0},
                        {0, 0, 1, D[3]}, 
                        {0, 0, 0, 1}
                        };
    
    Eigen::Matrix4d T54{
                        {cos(Th(4)),  -sin(Th(4)),  0, 0}, 
                        {0, 0, -1, -D[4]}, 
                        {sin(Th(4)),  cos(Th(4)), 0, 0},
                        {0, 0, 0, 1}
                        };
                

    Eigen::Matrix4d T65{
                        {cos(Th(5)),  -sin(Th(5)),  0, 0}, 
                        {0, 0, 1, D[5]}, 
                        {-sin(Th(5)),  -cos(Th(5)), 0, 0},
                        {0, 0, 0, 1}
                        };

    Eigen::Matrix4d T06 = ((T10*T21)*(T32*T43))*(T54*T65);

    directK direct;

    direct.pe = T06.block<3,1>(0,3);
    direct.Re = T06.block<3,3>(0,0);

    return direct;
}