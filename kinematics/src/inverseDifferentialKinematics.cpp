#include "inverseDifferentialKinematics.h"
#include "functions.h"
#include "directKinematics.h"
#include <iostream>
#include "kinematics.h"

Eigen::MatrixXd inverseDiffKinematics(Eigen::VectorXd th, Eigen::Vector3d endPos, Eigen::Vector3d endOrientation){
    
    directK currFwk = ur5DirectKinematics(th);
    Eigen::Matrix3d currRot = currFwk.Re;
    Eigen::Vector3d currPos = currFwk.pe;
    Eigen::Vector3d currRotEuler = rotmToAngleAxis(currRot);

    Eigen::MatrixXd joints_config = th;
    Eigen::Vector3d xd;
    Eigen::Vector3d xe;
    Eigen::Matrix3d Re;
    Eigen::Vector3d eule;
    Eigen::VectorXd qk = th;
    Eigen::VectorXd q = th;
    Eigen::VectorXd dotq;
    
    int iter = 0;
    Eigen::Vector3d distancePos = endPos - currPos;
    Eigen::Vector3d distanceOrient = endOrientation - currRotEuler;
    
    /* Check on the error from actual position to desired posiiton*/
    while( (distancePos.norm() > 0.001  || distanceOrient.norm() > 0.01 ) && iter < 1500){ 

        /* Computation of the direct kinematics to get the actual position and orientation of the end effector*/
        currFwk = ur5DirectKinematics(qk);
        Re = currFwk.Re;
        xe = currFwk.pe;
        eule = rotmToAngleAxis(Re);

        xd = desPos(xe, endPos);
        
        dotq = dotQ(qk, xe, xd, Re, endOrientation);
        
        /* Joint angles updated using the derivative multiplied by a small dt */
        qk = q + dotq * deltaT;
        q = qk;
        
        /* Saved the config. found */
        joints_config.conservativeResize(joints_config.rows(), joints_config.cols()+1);
        joints_config.block<6,1>(0, joints_config.cols()-1) = qk;

        distancePos = endPos - xe;
        distanceOrient = computeOrientationErrorW(Re, eulerToRotationMatrix(endOrientation));

        iter++;
    }

    return joints_config;
}


Eigen::MatrixXd inverseDiffKinematicsControlComplete(Vector6d th, Eigen::Vector3d endPos, Eigen::Vector3d endOrientation){
    Eigen::MatrixXd joints_config = th;
    double dt = 0.01;

    directK dk;
    dk= ur5DirectKinematics(th);
    Eigen::Vector3d xe0 = dk.pe;
    Eigen::Matrix3d Re0 = dk.Re;
    Eigen::Vector3d phied0 = rotmToAngleAxis(Re0);
    Eigen::Vector3d phied;
    Eigen::Vector3d xd;
    Eigen::Vector3d vd;
    Eigen::Vector3d xe;
    Eigen::Matrix3d Re;
    Eigen::Vector3d eule;
    Vector6d dotq;
    Eigen::Vector3d phidVar;
    Eigen::Vector3d phiddot;
    Eigen::Vector3d unitVec;
    Eigen::Vector3d errPos = endPos-xe0;
    Eigen::Vector3d erOrient = endOrientation-phied0;
    Vector6d qk = th;
    Vector6d q = qk;

    for(double t=0;t<10; t+=dt){
        dk= ur5DirectKinematics(qk);
        xe = dk.pe;
        Re = dk.Re;
        phied = rotmToAngleAxis(Re);

        
        xd = pd(t, endPos, xe);
        phidVar = phid(t, endOrientation, phied);
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
