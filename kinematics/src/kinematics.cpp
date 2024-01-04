#include "kinematics.h"
#include <math.h>
#include <cmath>
#include <ros_impedance_controller/generic_float.h>
#include "typedefs.h"
#include "inverseDifferentialKinematics.h"
#include "inverseKinematics.h"
#include "directKinematics.h"
#include "functions.h"
#include "typedefs.h"

void sendDesJstate(const Vector6d & joint_pos, const Eigen::Vector3d & gripper_pos){    
    /* GRIPPER MANAGEMENT */
    if(gripper_sim){
      int j=0;
      if(soft_gripper){
        jointState_msg_robot.data.resize(8);
        for (int i=joint_pos.size() ; i < joint_pos.size()+gripper_pos.size()-1 ; i++){
          jointState_msg_robot.data[i] = gripper_pos[j++];
        }
      }else{
        jointState_msg_robot.data.resize(9);
        for (int i=joint_pos.size() ; i < joint_pos.size()+gripper_pos.size(); i++){
          jointState_msg_robot.data[i] = gripper_pos[j++];
        }
      }
    }   

    /* JOINTS MANAGEMENT */
    for (int i = 0; i < joint_pos.size(); i++){
      jointState_msg_robot.data[i] = joint_pos[i];
    }
    pub.publish(jointState_msg_robot);
}

void moveGripper(const double diameter){
    ros_impedance_controller::generic_float::Request req;
    ros_impedance_controller::generic_float::Response res;

    req.data = diameter;

    gripperClient.call(req, res); 

    if(!res.ack){
        ROS_ERROR("GRIPPER FAIL");
    }
}

double mapToGripperJoints(double diameter){
    if(soft_gripper){
        int D0 = 40;
        int L = 60;
        double delta = 0.5*(diameter - D0);

        return atan2(delta, L);
    }else{
        return (diameter - 22) / (130 - 22) * (-M_PI) + M_PI;
    }
}

void setNewGripperPosition(double diameter){

    double q = mapToGripperJoints(diameter);

    Eigen::Vector3d v;
    v << q, q, q;

    setGripper(v);
    //this->gripper_diameter = diameter;
}

void move_gripper(double diameter){ //???
    if(real_robot){
        moveGripper(diameter);
    }else{
        setNewGripperPosition(diameter);
        sendDesJstate(jointsAndGripper.joints, jointsAndGripper.gripper);
    }
}

void setJoints(Vector6d q){
    jointsAndGripper.joints = q;
}

void setGripper(Eigen::Vector3d gripper){
    jointsAndGripper.gripper = gripper;
}

void setJointsAndGripper(Vector6d q, Eigen::Vector3d gripper){
    jointsAndGripper.joints = q;
    jointsAndGripper.gripper = gripper;
}

Eigen::VectorXd nearest_config(Eigen::VectorXd qk, Eigen::MatrixXd val){
    Eigen::VectorXd min_config = val.block<1,6>(0,0);
    Eigen::VectorXd diff_min = qk - min_config;
    Eigen::VectorXd diff;
    Eigen::VectorXd candidate;

    for(int i=1; i < val.rows(); i++){
        candidate = val.block<1,6>(i,0);
        diff = qk -candidate;
        diff << diff(0)*2, diff(1)*2, diff(2)*2, diff(3), diff(4), diff(5); // more weight on the first joints -> reduce shoulder movements
        if(diff.norm() < diff_min.norm()){    // looking for the nearest configuration
            min_config = candidate;
            diff_min = diff;
        }
    }

    return min_config;
}

Eigen::MatrixXd jointSpace_kinematics(Eigen::VectorXd qk, Eigen::Vector3d endPos, Eigen::Vector3d endOrient ){
    Eigen::MatrixXd val = ur5inverseKinematics(endPos, eulerToRotationMatrix(endOrient));
    Eigen::VectorXd endConfig = nearest_config(qk, val);
    /*to be added to gripper getter*/
   /*  for (int j = 0; j < qk.size(); j++) {
            qk(j) = atan2(std::imag(exp(1i * qk(j))), std::real(exp(1i * qk(j))));
    } */
    Eigen::VectorXd qNext = qk;
    Eigen::VectorXd error = endConfig - qNext;
    Eigen::MatrixXd joints_config = qk.transpose();


    // check if orientations are correct
    Eigen::Matrix3d rot;
    Eigen::Vector3d pos;
    directK tmp = ur5DirectKinematics(endConfig);
    rot = tmp.Re;
    pos = tmp.pe;

    /* fill the matrix with the middle configurations */
    int iter = 0;
    while (error.norm() > 0.005)
    {
        qNext = qNext + 5*deltaT* error/error.norm();
        joints_config.conservativeResize(joints_config.rows() + 1, joints_config.cols());
        joints_config.block<1,6>(joints_config.rows()-1, 0) = qNext.transpose();
        error = endConfig - qNext;
        iter++;
    }

    return joints_config;
}

void move(Eigen::Vector3d poseF, Eigen::Vector3d orientF){
    ros::Rate loop_rate(250.);
    readJoints();
    Eigen::MatrixXd traj = inverseDiffKinematicsControlComplete(jointsAndGripper.joints, poseF, orientF);
    //Eigen::MatrixXd traj = jointSpace_kinematics(jointsAndGripper.joints, poseF, orientF);
    
    int i;
    for(i=0; i<traj.cols(); i++){
        sendDesJstate(traj.block<6,1>(0,i), jointsAndGripper.gripper);
        ros::spinOnce();
        loop_time++;
        loop_rate.sleep();
    }
    setJoints(traj.block<6,1>(0,i-1));
} 

void move2(Eigen::Vector3d poseF, Eigen::Vector3d orientF){
    ros::Rate loop_rate(250.);
    readJoints();
    Eigen::MatrixXd traj = inverseDiffKinematicsControlCompleteAnglesAxis(jointsAndGripper.joints, poseF, orientF);
    //Eigen::MatrixXd traj = jointSpace_kinematics(jointsAndGripper.joints, poseF, orientF);
    
    int i;
    for(i=0; i<traj.cols(); i++){
        sendDesJstate(traj.block<6,1>(0,i), jointsAndGripper.gripper);
        ros::spinOnce();
        loop_time++;
        loop_rate.sleep();
    }
    setJoints(traj.block<6,1>(0,i-1));
} 

/* void initFilter(const Vector6d & joint_pos){
        filter_1 = joint_pos;
        filter_2 = joint_pos;
}

Vector6d secondOrderFilter(const Vector6d & input, const double rate, const double settling_time){
        double dt = 1 / rate;
        double gain =  dt / (0.1*settling_time + dt);
        filter_1 = (1 - gain) * filter_1 + gain * input;
        filter_2 = (1 - gain) * filter_2 + gain *filter_1;
        return filter_2;
} */

void readJoints(){
    sensor_msgs::JointState::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states");

    Vector9d joints;

    if(soft_gripper){
      joints << msg->position[4], msg->position[3], msg->position[0], msg->position[5], msg->position[6], msg->position[7], msg->position[1], msg->position[2], 0;
    }else{
      joints << msg->position[5], msg->position[4], msg->position[0], msg->position[6], msg->position[7], msg->position[8], msg->position[1], msg->position[2], msg->position[3];
    }

    jointsAndGripper.gripper = joints.block<3,1>(6,0);
    jointsAndGripper.joints = joints.block<6,1>(0,0); 
    //return joints;
}

Eigen::Vector3d worldToRobot(Eigen::Vector3d p){
    Eigen::Vector4d pe;
    pe << p(0), p(1), p(2), 1;

    Eigen::Matrix4d t0b {
        {1, 0, 0, -0.5},
        {0, -1, 0, 0.35},
        {0, 0, -1, 1.75},
        {0, 0,  0,  1}};

    return (t0b*pe).block<3,1>(0,0);
} 

Eigen::Vector3d robotToWorld(Eigen::Vector3d p){
    Eigen::Vector4d pe;
    pe << p(0), p(1), p(2), 1;

    Eigen::Matrix4d t0b {
        {1, 0, 0, 0.5},
        {0, -1, 0, 0.35},
        {0, 0, -1, 1.75},
        {0, 0, 0, 1}};

    return (t0b*pe).block<3,1>(0,0);
} 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinematics");
    ros::NodeHandle nH;

    nH.getParam("/real_robot", real_robot);
    nH.getParam("/soft_gripper", soft_gripper);
    nH.getParam("/gripper_sim", gripper_sim);

    pub = nH.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);

    ros::Rate loop_rate(loop_frequency);
    
    Vector6d q_des0 {-0.32, -0.78, -2.1, -1.63, -1.57,  3.49};
    ;
    readJoints();

    /* std::cout << "joints values" << jointsAndGripper.joints<< std::endl;
    std::cout << "gripper values" << jointsAndGripper.gripper<< std::endl;
    std::cout << "joint values should be " << q_des0<< std::endl; 
    FUNZIONA CON UN ERRORE NEL TERZO GIUNTO(PARI A QUELLO STAMPATO DA FOCCHI)
    

    std::cout << "Posizione del gripper a: " << ur5DirectKinematics(jointsAndGripper.joints).pe << std::endl;
    std::cout << "Posizione del gripper con base il mondo: " << robotToWorld(ur5DirectKinematics(jointsAndGripper.joints).pe) << std::endl;
    std::cout << "Rotazione del gripper a: " << ur5DirectKinematics(jointsAndGripper.joints).Re << std::endl;
    SEMBRA FUNZIONARE ANCHE TRASFORMANDO I VALORI RISPETTO AL MONDO
    */
    
    /* std::cout << "joints values" << jointsAndGripper.joints<< std::endl;
    directK dir = ur5DirectKinematics(jointsAndGripper.joints);
    std::cout << "Posizione del gripper a: " << dir.pe << std::endl;
    std::cout << "Rotazione del gripper a: " << dir.Re << std::endl;

    Matrix68d inv =  ur5inverseKinematics(dir.pe, dir.Re);
    std::cout << "Soluiozni  " << std::endl << inv << std::endl;
    
    Matrix66d J = jacobian(jointsAndGripper.joints); 
    SEMBRANO FUNZIONARE ANCHE QUESTE FUNZIONI
    */
   
   //move(worldToRobot({0.573892, 0.636202, 0.87}), {0.,0.,0.});
   move2(worldToRobot({0.573892, 0.636202, 0.87}), {0.,0.,0.});
  
  return 0;
}