#include "inverseDifferentialKinematics.h"
#include "functions.h"
#include "directKinematics.h"
#include <iostream>
#include "kinematics.h"

Eigen::MatrixXd inverseDiffKinematics(Eigen::VectorXd th, Eigen::Vector3d endPos, Eigen::Vector3d endOrientation){
    //std::cout << "prova 1" << std::endl;
    directK curr_fwk = ur5DirectKinematics(th);
    //std::cout << "prova 2" << std::endl;
    //std::cout << curr_fwk.Re << std::endl;
    //std::cout << curr_fwk.pe << std::endl;
    Eigen::Matrix3d curr_rot = curr_fwk.Re;
    Eigen::Vector3d curr_pos = curr_fwk.pe;
    Eigen::Vector3d curr_rot_euler = curr_rot.eulerAngles(0,1,2); 

    Eigen::MatrixXd joints_config = th.transpose();
    Eigen::Vector3d xd;
    Eigen::Vector3d xe;
    Eigen::Matrix3d Re;
    Eigen::Vector3d eule;
    Eigen::VectorXd qk = th;
    Eigen::VectorXd q = th;
    Eigen::VectorXd dotq;
    
    int iter = 0;
    Eigen::Vector3d distancePos = endPos - curr_pos;
    Eigen::Vector3d distanceOrient = endOrientation - curr_rot_euler;
    
    //std::cout << "prova 3" << std::endl;
    /* Check on error from actual pos. to desired pos. and number of iterations */
    while( (distancePos.norm() > 0.001  || distanceOrient.norm() > 0.01 ) && iter < 1500){ 
        //std::cout << "prova 4" << std::endl;
        /* Calc. direct kin. to know my actual position and orientation in space */
        curr_fwk = ur5DirectKinematics(th);
        Re = curr_fwk.Re;
        xe = curr_fwk.pe;
        eule = Re.eulerAngles(0,1,2); 
        //std::cout << "prova 5" << std::endl;

        xd = desPos(xe, endPos);
        //xd = computePositionError(xe, endPos);
        //std::cout << "prova 6" << std::endl;
        dotq = dotQ(qk, xe, xd, Re, endOrientation);
        //std::cout << "prova 7" << std::endl;
        
        /* Joint angles updated using the derivative (dotq) multiplied by a small step */
        qk = q + dotq * deltaT;
        //std::cout << "prova 7-2" << std::endl; //STUCK HERE
        q = qk;
        //std::cout << "prova 8" << std::endl;
        /* Save the config. found */
        joints_config.conservativeResize(joints_config.rows() + 1, joints_config.cols());
        //std::cout << "prova 9" << std::endl;
        joints_config.block<1,6>(joints_config.rows()-1, 0) = qk.transpose();
        //std::cout << "prova 10" << std::endl;

        distancePos = endPos - xe;
        //std::cout << "prova 11" << std::endl;
        distanceOrient = computeOrientationErrorW(Re, eulerToRotationMatrix(endOrientation));
        //sstd::cout << "prova 12" << std::endl;
        iter++;
    }
    
    std::cout << "numero iterazioni: " << iter << std::endl;

    return joints_config;
}

//(Vector6d q, Eigen::Vector3d xe, Eigen::Vector3d xd, Eigen::Vector3d vd, Eigen::Matrix3d Re, Eigen::Vector3d phid, Eigen::Vector3d phidDot)
Eigen::MatrixXd inverseDiffKinematicsControlComplete(Vector6d th, Eigen::Vector3d endPos, Eigen::Vector3d endOrientation){
    Eigen::MatrixXd joints_config = th;
    double dt = 0.1;

    directK dk;
    dk= ur5DirectKinematics(th);
    Eigen::Vector3d xe0 = dk.pe;
    Eigen::Matrix3d Re0 = dk.Re;
    Eigen::Vector3d phied0 = rotmToAngleAxis(Re0);
    Eigen::Vector3d xd;
    Eigen::Vector3d vd;
    Eigen::Vector3d xe;
    Eigen::Matrix3d Re;
    Eigen::Vector3d eule;
    Vector6d dotq;
    Eigen::Vector3d phidVar;
    Eigen::Vector3d phiddot;

    Vector6d qk = th;
    Vector6d q = qk;

    for(double t=0;t<10; t+=dt){
        dk= ur5DirectKinematics(qk);
        xe = dk.pe;
        Re = dk.Re;
        
        xd = pd(t, endPos, xe0);
        phidVar = phid(t, endOrientation, phied0);
        vd = ((pd(t, endPos, xe0)-pd((t-dt), endPos, xe0))/dt);
        phiddot = (phid(t, endOrientation, phied0)-phid((t-dt), endOrientation, phied0))/dt;
        
        dotq = dotQControlComplete(qk, xe, xd, vd, Re, phidVar, phiddot);
        qk = q + dotq * deltaT;
        q = qk;

        joints_config.conservativeResize(joints_config.rows(), joints_config.cols()+1);
        joints_config.block<6,1>(0, joints_config.cols()-1) = qk;
    }
    return joints_config;
}


Eigen::MatrixXd inverseDiffKinematicsControlCompleteAnglesAxis(Vector6d th, Eigen::Vector3d endPos, Eigen::Vector3d endOrientation){
    Eigen::MatrixXd joints_config = th;
    double dt = 0.002;

    directK dk;
    dk= ur5DirectKinematics(th);
    
    Eigen::Vector3d xe0 = dk.pe;
    Eigen::Matrix3d Re0 = dk.Re;
    Eigen::Vector3d phied0 = rotmToAngleAxis(Re0);
    Eigen::Vector3d xd;
    Eigen::Vector3d vd;
    Eigen::Vector3d xe;
    Eigen::Matrix3d Re;
    Eigen::Vector3d eule;
    Vector6d dotq;
    Eigen::Vector3d phidVar;
    Eigen::Vector3d phiddot;
    Vector6d qk = th;
    Vector6d q = qk;

    for(double t=0;t<1; t+=dt){
        dk= ur5DirectKinematics(qk);
        xe = dk.pe;
        Re = dk.Re;
        
        xd = pd(t, endPos, xe0);
        phidVar = phid(t, endOrientation, phied0);
        vd = ((pd(t, endPos, xe0)-pd((t-dt), endPos, xe0))/dt);
        phiddot = (phid(t, endOrientation, phied0)-phid((t-dt), endOrientation, phied0))/dt;
        
        dotq = dotQControlCompleteAangleAxis(qk, xe, xd, vd, Re, phidVar, phiddot);  
        qk = q + dotq * deltaT;
        q = qk;

        joints_config.conservativeResize(joints_config.rows(), joints_config.cols()+1);
        joints_config.block<6,1>(0, joints_config.cols()-1) = qk;
    }
    return joints_config;   
}