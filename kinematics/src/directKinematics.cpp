#include "directKinematics.h"

/**
 * @brief Calculates the general transformation matrix for a given joint angle.
 *
 * This function computes the 4x4 transformation matrix for a specific joint angle, 
 * given the Denavit-Hartenberg parameters.
 *
 * @param th The joint angle vector.
 * @param i The index of the joint.
 * @return Eigen::Matrix4d The 4x4 transformation matrix.
 */

Eigen::Matrix4d generalTransformationMatrix(Vector6d th,int i){
    return Eigen::Matrix4d{
    {cos(th(i)), -sin(th(i))*cos(Alpha(i)), sin(th(i))*sin(Alpha(i)), A(i)*cos(th(i))},
    {sin(th(i)), cos(th(i))*cos(Alpha(i)), -cos(th(i))*sin(Alpha(i)), A(i)*sin(th(i))},
    {0, sin(Alpha(i)), cos(Alpha(i)), D(i)},
    {0, 0, 0, 1}
    };
}

/**
 * @brief Calculates the direct kinematics of a UR5 robot.
 *
 * This function computes the end-effector position and orientation (direct kinematics)
 * for a given set of joint angles of a UR5 robot.
 *
 * @param Th Vector6d representing the joint angles.
 * @return directK A structure containing the end-effector position (pe) and orientation (Re).
 */

directK ur5DirectKinematics(Vector6d Th){
    directK direct;
    
    Eigen::Matrix4d T10 = generalTransformationMatrix(Th, 0);
    Eigen::Matrix4d T21 = generalTransformationMatrix(Th, 1);
    Eigen::Matrix4d T32 = generalTransformationMatrix(Th, 2);
    Eigen::Matrix4d T43 = generalTransformationMatrix(Th, 3);
    Eigen::Matrix4d T54 = generalTransformationMatrix(Th, 4);
    Eigen::Matrix4d T65 = generalTransformationMatrix(Th, 5);

    Eigen::Matrix4d T06 = (T10*T21*T32*T43*T54*T65);

    direct.pe = T06.block<3,1>(0,3);
    direct.Re = T06.block<3,3>(0,0);

    return direct;
}

/* directK ur5DirectKinematics(Vector6d Th){
    directK direct;
    
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

    Eigen::Matrix4d T06 = (T10*T21*T32*T43*T54*T65);

    direct.pe = T06.block<3,1>(0,3);
    direct.Re = T06.block<3,3>(0,0);

    return direct;
} */