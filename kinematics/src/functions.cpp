#include "functions.h"
#include "kinematics.h"
#include "directKinematics.h"
#include <cmath>
#include <iostream>
#include <string>

/* Matrix66d jacobian(Vector6d th){
    Matrix66d J;
    Vector6d J1(6), J2(6), J3(6), J4(6), J5(6), J6(6);


    J1 << D[4]*(cos(th(0))*cos(th(4)) + cos(th(1)+th(2)+th(3))*sin(th(0))*sin(th(4))) + D[2]*cos(th(0)) + D[3]*cos(th(0)) - A[3]*cos(th(1) + th(2))*sin(th(0)) - A[2]*cos(th(1))*sin(th(0)) - D[4]*sin(th(1)+th(2)+th(3))*sin(th(0)),
        D[4]*(cos(th(4))*sin(th(0)) - cos(th(1)+th(2)+th(3))*cos(th(0))*sin(th(4))) + D[2]*sin(th(0)) + D[3]*sin(th(0)) + A[3]*cos(th(1)+th(2))*cos(th(0)) + A[2]*cos(th(0))*cos(th(1)) + D[4]*sin(th(1)+th(2)+th(3))*cos(th(0)),
        0, 0, 0, 1;

    J2 << -cos(th(0))*(A[3]*sin(th(1)+th(2)) + A[2]*sin(th(1)) + D[4]*(sin(th(1)+th(2))*sin(th(3)) - cos(th(1) + th(2))*cos(th(3)) ) - D[4]*sin(th(4))*(cos(th(1)+th(2))*sin(th(3)) + sin(th(1)+th(2))*cos(th(3)) ) ),
        -sin(th(0))*(A[3]*sin(th(1) + th(2)) + A[2]*sin(th(1)) + D[4]*(sin(th(1) + th(2))*sin(th(3)) - cos(th(1) + th(2))*cos(th(3))) - D[4]*sin(th(4))*(cos(th(1) + th(2))*sin(th(3)) + sin(th(1) + th(2))*cos(th(3)))),
        A[3]*cos(th(1) + th(2)) - (D[4]*sin(th(1) + th(2) + th(3) + th(4)))/2 + A[2]*cos(th(1)) + (D[4]*sin(th(1) + th(2) + th(3) - th(4)))/2 + D[4]*sin(th(1) + th(2) + th(3)),
        sin(th(0)),
        -cos(th(0)),
        0;

    J3 << cos(th(0))*(D[4]*cos(th(1) + th(2) + th(3)) - A[3]*sin(th(1) + th(2)) + D[4]*sin(th(1) + th(2) + th(3))*sin(th(4))),
        sin(th(0))*(D[4]*cos(th(1) + th(2) + th(3)) - A[3]*sin(th(1) + th(2)) + D[4]*sin(th(1) + th(2) + th(3))*sin(th(4))),
        A[3]*cos(th(1) + th(2)) - (D[4]*sin(th(1) + th(2) + th(3) + th(4)))/2 + (D[4]*sin(th(1) + th(2) + th(3) - th(4)))/2 + D[4]*sin(th(1) + th(2) + th(3)),
        sin(th(0)),
        -cos(th(0)),
        0;

    J4 << D[4]*cos(th(0))*(cos(th(1) + th(2) + th(3)) + sin(th(1) + th(2) + th(3))*sin(th(4))),
        D[4]*sin(th(0))*(cos(th(1) + th(2) + th(3)) + sin(th(1) + th(2) + th(3))*sin(th(4))),
        D[4]*(sin(th(1) + th(2) + th(3) - th(4))/2 + sin(th(1) + th(2) + th(3)) - sin(th(1) + th(2) + th(3) + th(4))/2),
        sin(th(0)),
        -cos(th(0)),
        0;
    
    J5 << -D[4]*sin(th(0))*sin(th(4)) - D[4]*cos(th(1) + th(2) + th(3))*cos(th(0))*cos(th(4)),
     D[4]*cos(th(0))*sin(th(4)) - D[4]*cos(th(1) + th(2) + th(3))*cos(th(4))*sin(th(0)),
     -D[4]*(sin(th(1) + th(2) + th(3) - th(4))/2 + sin(th(1) + th(2) + th(3) + th(4))/2),
     sin(th(1) + th(2) + th(3))*cos(th(0)),
     sin(th(1) + th(2) + th(3))*sin(th(0)),
    -cos(th(1) + th(2) + th(3));

    J6 << 0,0,0,
        cos(th(4))*sin(th(0)) - cos(th(1) + th(2) + th(3))*cos(th(0))*sin(th(4)),
        -cos(th(0))*cos(th(4)) - cos(th(1) + th(2) + th(3))*sin(th(0))*sin(th(4)),
        -sin(th(1) + th(2) + th(3))*sin(th(4));

    J.block<6,1>(0,0) = J1; 
    J.block<6,1>(0,1) = J2; 
    J.block<6,1>(0,2) = J3; 
    J.block<6,1>(0,3) = J4; 
    J.block<6,1>(0,4) = J5; 
    J.block<6,1>(0,5) = J6; 

    return J;
} */

Matrix66d jacobian(Vector6d Th){
    Matrix66d J;
    Vector6d J1, J2, J3, J4, J5, J6;
    double D1 = D(0), D2 = D(1), D3 = D(2), D4 = D(3), D5 = D(4),  D6 = D(5);
    double A1 = A(0), A2 = A(1), A3 = A(2), A4 = A(3), A5 = A(4),  A6 = A(5);
    double th1 = Th(0), th2 = Th(1), th3 = Th(2), th4 = Th(3), th5 = Th(4),th6 = Th(5);
    
    J1 << D5*(cos(th1)*cos(th5) + cos(th2 + th3 + th4)*sin(th1)*sin(th5)) + D4*cos(th1) - A2*cos(th2)*sin(th1) - D5*sin(th2 + th3 + th4)*sin(th1) - A3*cos(th2)*cos(th3)*sin(th1) + A3*sin(th1)*sin(th2)*sin(th3),
          D5*(cos(th5)*sin(th1) - cos(th2 + th3 + th4)*cos(th1)*sin(th5)) + D4*sin(th1) + A2*cos(th1)*cos(th2) + D5*sin(th2 + th3 + th4)*cos(th1) + A3*cos(th1)*cos(th2)*cos(th3) - A3*cos(th1)*sin(th2)*sin(th3),
          0,
          0,
          0,
          1;

    J2 << -cos(th1)*(A3*sin(th2 + th3) + A2*sin(th2) + D5*(sin(th2 + th3)*sin(th4) - cos(th2 + th3)*cos(th4)) - D5*sin(th5)*(cos(th2 + th3)*sin(th4) + sin(th2 + th3)*cos(th4))),
          -sin(th1)*(A3*sin(th2 + th3) + A2*sin(th2) + D5*(sin(th2 + th3)*sin(th4) - cos(th2 + th3)*cos(th4)) - D5*sin(th5)*(cos(th2 + th3)*sin(th4) + sin(th2 + th3)*cos(th4))), 
          A3*cos(th2 + th3) - (D5*sin(th2 + th3 + th4 + th5))/2 + A2*cos(th2) + (D5*sin(th2 + th3 + th4 - th5))/2 + D5*sin(th2 + th3 + th4),
          sin(th1),
          -cos(th1),
          0;
    J3 << cos(th1)*(D5*cos(th2 + th3 + th4) - A3*sin(th2 + th3) + D5*sin(th2 + th3 + th4)*sin(th5)),
          sin(th1)*(D5*cos(th2 + th3 + th4) - A3*sin(th2 + th3) + D5*sin(th2 + th3 + th4)*sin(th5)),
          A3*cos(th2 + th3) - (D5*sin(th2 + th3 + th4 + th5))/2 + (D5*sin(th2 + th3 + th4 - th5))/2 + D5*sin(th2 + th3 + th4),
          sin(th1),
          -cos(th1),
          0;
    J4 << D5*cos(th1)*(cos(th2 + th3 + th4) + sin(th2 + th3 + th4)*sin(th5)),
          D5*sin(th1)*(cos(th2 + th3 + th4) + sin(th2 + th3 + th4)*sin(th5)),
          D5*(sin(th2 + th3 + th4 - th5)/2 + sin(th2 + th3 + th4) - sin(th2 + th3 + th4 + th5)/2),
          sin(th1),
          -cos(th1),
          0;
    J5 << D5*cos(th1)*cos(th2)*cos(th5)*sin(th3)*sin(th4) - D5*cos(th1)*cos(th2)*cos(th3)*cos(th4)*cos(th5) - D5*sin(th1)*sin(th5) + D5*cos(th1)*cos(th3)*cos(th5)*sin(th2)*sin(th4) + D5*cos(th1)*cos(th4)*cos(th5)*sin(th2)*sin(th3),
          D5*cos(th1)*sin(th5) + D5*cos(th2)*cos(th5)*sin(th1)*sin(th3)*sin(th4) + D5*cos(th3)*cos(th5)*sin(th1)*sin(th2)*sin(th4) + D5*cos(th4)*cos(th5)*sin(th1)*sin(th2)*sin(th3) - D5*cos(th2)*cos(th3)*cos(th4)*cos(th5)*sin(th1),
          -D5*(sin(th2 + th3 + th4 - th5)/2 + sin(th2 + th3 + th4 + th5)/2),
          sin(th2 + th3 + th4)*cos(th1),
          sin(th2 + th3 + th4)*sin(th1),
          -cos(th2 + th3 + th4);
    J6 << 0,
          0,
          0,
          cos(th5)*sin(th1) - cos(th2 + th3 + th4)*cos(th1)*sin(th5),
          -cos(th1)*cos(th5) - cos(th2 + th3 + th4)*sin(th1)*sin(th5),
          -sin(th2 + th3 + th4)*sin(th5);

    /* J.col(0) << J1;
    J.col(1) << J2;
    J.col(2) << J3;
    J.col(3) << J4;
    J.col(4) << J5;
    J.col(5) << J6; */
    J << J1, J2, J3, J4, J5, J6;

    return J;
}

Eigen::Vector3d attrForce_pos(Eigen::Vector3d error){
    return -0.001*(error)/error.norm();
}

Eigen::Vector3d repulForce(Eigen::Vector3d xe){

    double distance = sqrt(xe(0)*xe(0) + xe(1)*xe(1));  // distance from the point below the shoudler of the robot (x=0, y=0)
    if(distance < 0.2){
        Eigen::Vector3d pot = 0.001*(xe/distance);
        pot(2) = 0;
        return pot;

    }else{
        
        return Eigen::MatrixXd::Zero(3,1);
    }
}

Eigen::Vector3d desPos(Eigen::Vector3d xe, Eigen::Vector3d xf){

    Eigen::Vector3d errPos = xe - xf;

    if(errPos.norm() > 0.0001){
        if(errPos.norm() > 0.1){
            return xe + (attrForce_pos(errPos) + repulForce(xe));
        }else{
            return xe - 0.03*errPos;
        }
    }

    return xe;
}

Eigen::VectorXd dotQ(Vector6d qk, Eigen::Vector3d xe, Eigen::Vector3d xd, Eigen::Matrix3d Re, Eigen::Vector3d phid){
    Eigen::MatrixXd Jac;
    double Kp = 0.8;
    double Kphi = 6;
    Eigen::VectorXd V(6);
    Eigen::Matrix3d w_R_d;
    Eigen::Vector3d errorOrientation;
    Eigen::Vector3d errorPosition;

    w_R_d = eulerToRotationMatrix(phid);
    errorOrientation = computeOrientationErrorW(Re, w_R_d);
    errorPosition = computePositionError(xe, xd);
    
    Jac = jacobian(qk);

    V.block<3,1>(0,0) = Kp*errorPosition/deltaT;
    V.block<3,1>(3,0) = Kphi*(errorOrientation.normalized());

    return (Jac + Eigen::MatrixXd::Identity(6,6)*(0.001)).inverse()*V;
}

Eigen::Vector3d pd(double t, Eigen::Vector3d xef, Eigen::Vector3d xe0){
    Eigen::Vector3d xd;
    double Tm  = 1.;
    double tb = t/10;
    if(tb>Tm){
        return xd = xef;
    }else{
        return xd = (tb)*xef + (1-tb)*xe0;
    }
}

Eigen::Vector3d phid(double t, Eigen::Vector3d phief, Eigen::Vector3d phie0){
    Eigen::Vector3d phid;
    double Tm  = 1.;
    double tb = t/10;
    if(tb>Tm){
        return phid = phief;
    }else{
        return phid = (tb)*phief + (1-tb)*phie0;
    }
}

Eigen::Vector3d rotmToAngleAxis(Eigen::Matrix3d R) {
    Eigen::Vector3d l;
    l << R(2,1)-R(1,2),
         R(0,2)-R(2,0),
         R(1,0)-R(0,1);
    Eigen::Vector3d alphaR;
    double l_norm = l.norm();

    if (l_norm == 0) {
        // R is a diagonal vector
        if (R(0,0)==1 && R(1,1)==1 && R(2,2)==1) {
            // R is the identity matrix
            alphaR << 0, 0, 0;
        } else {
            alphaR << R(0,0)+1, R(1,1)+1, R(2,2)+1;
            alphaR = alphaR * M_PI / 2;
        }
    } else {
        alphaR = l*atan2(l_norm, R(0,0)+R(1,1)+R(2,2)-1)/l_norm;
    }
    return alphaR;
}

Vector6d dotQControlComplete(Vector6d q, Eigen::Vector3d xe, Eigen::Vector3d xd, Eigen::Vector3d vd, Eigen::Matrix3d Re, Eigen::Vector3d phid, Eigen::Vector3d phidDot){
    Vector6d dotQ;    
    Matrix66d Jac;
    Eigen::Vector3d phie;
    
    Eigen::Matrix3d Kp = 27*Eigen::MatrixXd::Identity(3,3);
    Eigen::Matrix3d Kphi = 27*Eigen::MatrixXd::Identity(3,3);
    
    directK dk = ur5DirectKinematics(q);
    phie = rotMatToEuler(dk.Re); // phie = dk.Re.eulerAngles(2,1,0);
    Jac = jacobian(q);
    double gamma = phie(0);
    double beta = phie(1);
    double alpha = phie(2);
    
    Eigen::Matrix3d T {
        {cos(beta)*cos(gamma), -sin(gamma), 0},
        {cos(beta)*sin(gamma), cos(gamma), 0},
        {-sin(beta), 0, 1}
    };
    
    if((abs(T.determinant()))<1e-3){
        std::cout << "Near singular configuration" << std::endl;
    }

    Matrix66d Ta;
    Ta.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3);
    Ta.block<3,3>(0,3) = Eigen::MatrixXd::Zero(3,3);
    Ta.block<3,3>(3,0) = Eigen::MatrixXd::Zero(3,3);
    Ta.block<3,3>(3,3) = T;
    
    Matrix66d Ja = Ta.inverse()*Jac;
    
    Vector6d V;
    V.block<3,1>(0,0) = vd+Kp*(xd-xe);
    V.block<3,1>(3,0) = phidDot+Kphi*(phid-phie);
    
    dotQ = (Ja).inverse()*V;
    
    return dotQ;
}

Vector6d dotQControlCompleteAangleAxis(Vector6d q, Eigen::Vector3d xe, Eigen::Vector3d xd, Eigen::Vector3d vd, Eigen::Matrix3d Re, Eigen::Vector3d phid, Eigen::Vector3d phidDot){
    Vector6d dotQ;    
    Matrix66d Jac;
    Eigen::Vector3d phie;
    
    Eigen::Matrix3d Kp = 0.8*Eigen::MatrixXd::Identity(3,3);
    Eigen::Matrix3d Kphi = 6*Eigen::MatrixXd::Identity(3,3);
    Eigen::Matrix3d Red = eulerToRotationMatrix(phid);
    Eigen::Vector3d oError = computeOrientationErrorW(Re, Red);

    Jac = jacobian(q);

    directK dk = ur5DirectKinematics(q);
    phie = rotmToAngleAxis(dk.Re); //phie = dk.Re.eulerAngles(2,1,0);
    double psid = phid(0); //psi
    double thetad = phid(1); //theta
    double phidD = phid(2); //phi
    
    Eigen::Matrix3d T {
        {cos(thetad)*cos(phidD), -sin(phidD), 0},
        {cos(thetad)*sin(phidD), cos(phidD), 0},
        {-sin(thetad), 0, 1}
    };
    
    Eigen::Vector3d omedaDot = T*phidDot;
    
    Vector6d V;
    V.block<3,1>(0,0) = vd+Kp*(xd-xe);
    V.block<3,1>(3,0) = omedaDot+Kphi*oError;
    
    dotQ = (Jac+Eigen::MatrixXd::Identity(6,6)*(0.001)).inverse()*V;
    return dotQ;
}

Eigen::Vector3d computeOrientationErrorW(Eigen::Matrix3d w_R_e, Eigen::Matrix3d w_R_d){
        Eigen::Vector3d error; 
        //computaion of the relative orientation 
        Eigen::Matrix3d e_R_d = w_R_e.transpose()*w_R_d;

        double cosDtheta = (e_R_d(0,0) + e_R_d(1,1) + e_R_d(2,2) - 1)/2;
        Eigen::Vector3d util; 
        util << e_R_d(2,1) - e_R_d(1,2), e_R_d(0,2) - e_R_d(2,0), e_R_d(1,0) - e_R_d(0,1); 
        double sinDtheta = util.norm()*0.5;

        double dtheta = atan2(sinDtheta, cosDtheta);

        if(dtheta == 0){
            error = {0,0,0};
        }else{
            Eigen::Vector3d axis = 1/(2*sinDtheta)*util;
        error = w_R_e * axis * dtheta;
        }

        return error;
}

Eigen::Vector3d computePositionError(Eigen::Vector3d xe, Eigen::Vector3d xd){

    Eigen::Vector3d error = xd - xe;
    
    if(error.norm()>0.1){
        error = 0.1*error.normalized();
    }

    return error;
}

Eigen::Matrix3d eulerToRotationMatrix(Eigen::Vector3d euler){

    double psi = euler(0);    //yaw
    double theta = euler(1);  //pitch
    double phi = euler(2);    //roll

    Eigen::Matrix3d R;

    R << cos(phi)*cos(theta), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi),
         sin(phi)*cos(theta), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),
         -sin(theta), cos(theta)*sin(psi), cos(theta)*cos(psi);

    return R;

}

Eigen::Vector3d rotMatToEuler(Eigen::Matrix3d rotMatrix){
    double sy = sqrt(pow(rotMatrix(0,0),2) + pow(rotMatrix(1,0),2));

    bool singular = sy < 1e-6;
    double x, y, z;

    if(!singular){
        x = atan2(rotMatrix(2,1), rotMatrix(2,2));
        y = atan2(-rotMatrix(2,0), sy);
        z = atan2(rotMatrix(1,0), rotMatrix(0,0));
    }else{
        x = atan2(-rotMatrix(1,2), rotMatrix(1,1));
        y = atan2(-rotMatrix(2,0), sy);
        z = 0;
    }

    return Eigen::Vector3d(x,y,z);
}

Matrix66d pseudoInverse(Matrix66d J){
    Matrix66d pseudoInverse;

    pseudoInverse = J.transpose()*(J*J.transpose()).inverse();

    return pseudoInverse;
}

/*
void straightenBlockUp(Eigen::Vector3d pos, Eigen::Vector3d rot, std::string blockName){
    double roll = rot(2);
    double pitch = rot(1);
    double yaw = rot(0);
    double workingH = 1.1;
    double graspingH = 0.89;
    double releasingH = 0.92;

    bool reversed = false;
    
    if( (abs(roll) < 0.5 && abs(pitch) < 0.5) || ((0.9 < (abs(pitch)/M_PI) && (abs(pitch)/M_PI) < 1.1) && (0.9 < (abs(roll)/M_PI) && (abs(roll)/M_PI) < 1.1)) ){
        std::cout << "Block already up right! " << std::endl;
    }else{

        // upside down 
        if( ((0.9 < (abs(roll)/M_PI) && (abs(roll)/M_PI) < 1.1)  ||  (0.9 < (abs(pitch)/M_PI) && (abs(pitch)/M_PI) < 1.1) ) && !(0.9 < (abs(pitch)/M_PI_2) && (abs(pitch)/M_PI_2) < 1.1) ){
            
            move({pos(0), pos(1), workingH}, {0, 0, -yaw});

            move_gripper(50);

            move({pos(0), pos(1), graspingH}, {0, 0, -yaw});

            attach("ur5", "hand_1_link", blockName, "link");

            move({pos(0), pos(1), workingH}, {0, 0, 0}); 
            rotate({pos(0), pos(1), workingH}, {M_PI_2, M_PI, 0});
            move({pos(0), pos(1), releasingH}, {M_PI_2, M_PI, 0}); 

            detach("ur5", "hand_1_link", blockName, "link");
            move({pos(0), pos(1), workingH}, {M_PI_2, M_PI, 0});
            rotate({pos(0), pos(1), workingH}, {0,0,0});
            
            reversed = true;
        }else if( (0.9 < (abs(pitch)/M_PI_2) && (abs(pitch)/M_PI_2) < 1.1) ){ // lying on the side 
            move_gripper(75);

           if( pitch > 0 ){
                if( (roll<0 && yaw<0) || (roll>0 && yaw>0) ){
                    move({pos(0), pos(1), workingH}, {0, 0, -(roll-yaw)});
                    

                    if( roll-yaw < 1.57 && roll-yaw > -1.57 ) {

                        move({pos(0), pos(1), graspingH}, {0, 0, -(roll-yaw)});
                    }else{

                        move({pos(0), pos(1), graspingH}, {0, 0, -(roll-yaw)});
                        reversed = true;
                    }

                }else{
                    move({pos(0), pos(1), workingH}, {0, 0, -(roll+yaw)});

                    if( (roll+yaw < 1.57 && roll+yaw > -1.57) || abs(roll+yaw)>4.71 ){

                        move({pos(0), pos(1), graspingH}, {0, 0, -(roll+yaw)});
                    }else{

                        move({pos(0), pos(1), graspingH}, {0, 0, -(roll+yaw)});
                        reversed = true;
                    }

                }

           }else{
                move({pos(0), pos(1), workingH}, {0, 0, -(roll+yaw)});

                if( (roll+yaw < 1.57 && roll+yaw > -1.57) ){
                    move({pos(0), pos(1), graspingH}, {0, 0, -(roll+yaw)});
                    reversed = true;
                }else{
                    move({pos(0), pos(1), graspingH}, {0, 0, -(roll+yaw)});
                }

           }

            attach("ur5", "hand_1_link", blockName, "link");
            move({pos(0), pos(1), workingH}, {0, 0, 0});
            rotate({pos(0), pos(1), workingH}, {-M_PI_2, 0, M_PI_2});
            move({pos(0), pos(1), releasingH}, {-M_PI_2, 0, M_PI_2});  

        
            detach("ur5", "hand_1_link", handled_model.c_str(), "link");
            move({pos(0), pos(1), workingH}, {-M_PI_2, 0, M_PI_2});
            rotate({pos(0), pos(1), workingH}, {0,0,0});

        }else{
            cout << "Block Position & Orientation NOT Handle, Aborting..." << endl;
            return;
        }


        if(reversed == true){
                move({pos(0), pos(1), releasingH}, {0,0,0});
                attach("ur5", "hand_1_link", blockName, "link");

                move({pos(0), pos(1), workingH}, {0,0,0});
                rotate({pos(0), pos(1), workingH}, {0,0,M_PI});
                move({pos(0), pos(1), releasingH}, {0,0,M_PI});
                detach("ur5", "hand_1_link", blockName, "link");

                move({pos(0), pos(1), workingH}, {0,0,M_PI});
            }

        rotate({pos(0), pos(1), workingH}, {M_PI_2,M_PI,0});
        move({pos(0), pos(1), releasingH}, {M_PI_2,M_PI,0});

        attach("ur5", "hand_1_link", blockName, "link");
        move({pos(0), pos(1), workingH}, {M_PI_2,M_PI,0});
        rotate({pos(0), pos(1), workingH}, {0,0,0});
        move({pos(0), pos(1), releasingH}, {0,0,0});
        
        detach("ur5", "hand_1_link", blockName, "link");
        move({pos(0), pos(1), workingH}, {0,0,0});
    }
}
 */
