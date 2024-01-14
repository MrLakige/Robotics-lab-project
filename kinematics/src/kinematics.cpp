#include <unistd.h> 
#include "kinematics.h"
#include <math.h>
#include <cmath>
#include <ros_impedance_controller/generic_float.h>
#include "typedefs.h"
#include "inverseDifferentialKinematics.h"
#include "inverseKinematics.h"
#include "directKinematics.h"
#include "functions.h"
#include <ros/ros.h>
#include "gazebo_ros_link_attacher/Attach.h"
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

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


void setNewGripperPosition(double diameter){
    double q = mapToGripperJoints(diameter);

    Eigen::Vector3d v;
    v << q, q, q;

    setGripper(v);
}

void attachBlock(const char* model1, const char* link1, const char* model2, const char* link2){
    gazebo_ros_link_attacher::Attach::Request req;
    gazebo_ros_link_attacher::Attach::Response res;
    
    req.model_name_1 = model1;
    req.link_name_1 = link1;
    req.model_name_2 = model2;
    req.link_name_2 = link2;
    
    if (!attach_client.call(req, res)){
        ROS_INFO_STREAM("Attach failed");
    }
}

void detachBlock(const char* model1, const char* link1, const char* model2, const char* link2){
    gazebo_ros_link_attacher::Attach::Request req;
    gazebo_ros_link_attacher::Attach::Response res;
    
    req.model_name_1 = model1;
    req.link_name_1 = link1;
    req.model_name_2 = model2;
    req.link_name_2 = link2;
    
    if (!detach_client.call(req, res)){
        ROS_INFO_STREAM("Detach failed");
    }
}

void move_gripper(double diameter){ 
    if(real_robot){
        moveGripper(diameter);
    }else{
        setNewGripperPosition(diameter);
        sendDesJstate(jointsAndGripper.joints, jointsAndGripper.gripper);
    }
}

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
}

void move(Eigen::Vector3d poseF, Eigen::Vector3d orientF){
    ros::Rate loop_rate(250.);
    readJoints();
    Eigen::MatrixXd traj = inverseDiffKinematics(jointsAndGripper.joints, poseF, orientF);
    
    int i;
    for(i=0; i<traj.cols(); i++){
        sendDesJstate(traj.block<6,1>(0,i), jointsAndGripper.gripper);
        ros::spinOnce();
        loop_rate.sleep();
    }
    setJoints(traj.block<6,1>(0,i-1));
} 

/* void move2(Eigen::Vector3d poseF, Eigen::Vector3d orientF){
    ros::Rate loop_rate(250.);
    readJoints();
    Eigen::MatrixXd traj = inverseDiffKinematicsControlCompleteAnglesAxis(jointsAndGripper.joints, poseF, orientF);
    
    int i;
    for(i=0; i<traj.cols(); i++){
        sendDesJstate(traj.block<6,1>(0,i), jointsAndGripper.gripper);
        ros::spinOnce();
        loop_time++;
        loop_rate.sleep();
    }
    setJoints(traj.block<6,1>(0,i-1));
} 


void move3(Eigen::Vector3d poseF, Eigen::Vector3d orientF){
    ros::Rate loop_rate(250.);
    readJoints();
    Eigen::MatrixXd traj = inverseDiffKinematicsControlComplete(jointsAndGripper.joints, poseF, orientF);
    
    int i;
    for(i=0; i<traj.cols(); i++){
        sendDesJstate(traj.block<6,1>(0,i), jointsAndGripper.gripper);
        ros::spinOnce();
        loop_rate.sleep();
    }
    setJoints(traj.block<6,1>(0,i-1));
} */

Eigen::Vector3d transformWrldToRbt(Eigen::Vector3d p){
    Eigen::Vector4d pe;
    pe << p(0), p(1), p(2), 1;

    Eigen::Matrix4d t0b {
        {1, 0, 0, -0.5},
        {0, -1, 0, 0.35},
        {0, 0, -1, 1.75},
        {0, 0,  0,  1}};

    return (t0b*pe).block<3,1>(0,0);
} 

Eigen::Vector3d transformRbtToWrld(Eigen::Vector3d p){
    Eigen::Vector4d pe;
    pe << p(0), p(1), p(2), 1;

    Eigen::Matrix4d t0b {
        {1, 0, 0, 0.5},
        {0, -1, 0, 0.35},
        {0, 0, -1, 1.75},
        {0, 0, 0, 1}};

    return (t0b*pe).block<3,1>(0,0);
}

blockPose getBlockPose(std::string blockName){
    blockPose pos;

    if(strcmp(blockName.c_str(),"X1-Y1-Z2")==0){
        pos.x = 1.059747;
        pos.y = 0.266603;
        pos.z = 0.863913;
    }
    if(strcmp(blockName.c_str(),"X1-Y2-Z1")==0){
        pos.x = 0.903177;
        pos.y = 0.281704;
        pos.z = 0.863913;
    }
    if(strcmp(blockName.c_str(),"X1-Y2-Z2")==0){
        pos.x = 0.977510;
        pos.y = 0.582337;
        pos.z = 0.863913;
    }     
    if(strcmp(blockName.c_str(),"X1-Y2-Z2-CHAMFER")==0){
        pos.x = 0.896607;
        pos.y = 0.592261;
        pos.z = 0.863913;
    }       
    if(strcmp(blockName.c_str(),"X1-Y2-Z2-TWINFILLET")==0){
        pos.x = 0.983954;
        pos.y = 0.280836;
        pos.z = 0.863913;
    }       
    if(strcmp(blockName.c_str(),"X1-Y3-Z2")==0){
        pos.x = 0.899273;
        pos.y = 0.418322;
        pos.z = 0.863913;
    }
    if(strcmp(blockName.c_str(),"X1-Y3-Z2-FILLET")==0){
        pos.x = 1.059748;
        pos.y = 0.592607;
        pos.z = 0.863913;
    }
    if(strcmp(blockName.c_str(),"X1-Y4-Z1")==0){
        pos.x = 1.055354;
        pos.y = 0.419024;
        pos.z = 0.863913;
    }
    if(strcmp(blockName.c_str(),"X1-Y4-Z2")==0){
        pos.x = 0.980792;
        pos.y = 0.411519;
        pos.z = 0.863913;
    }
    if(strcmp(blockName.c_str(),"X2-Y2-Z2")==0){
        pos.x = 0.916188;
        pos.y = 0.717653;
        pos.z = 0.863913;
    }
    if(strcmp(blockName.c_str(),"X2-Y2-Z2-FILLET")==0){
        pos.x = 1.022507;
        pos.y = 0.714191;
        pos.z = 0.863913;
    }
    return pos;
}

void moveStrightObjFromTo(Eigen::Vector3d posI, Eigen::Vector3d posF, const char* blockName){
    move(transformWrldToRbt({posI(0), posI(1), workingH}), {0.,0.,0.});
    move_gripper(60); 
    move(transformWrldToRbt({posI(0), posI(1), releasingH}), {0.,0.,0.});
    attachBlock("ur5", "wrist_3_link", blockName, "link");
    move_gripper(27);
    move(transformWrldToRbt({posI(0), posI(1), workingH}), {0.,0.,0.});
    move(transformWrldToRbt({posF(0), posF(1), workingH}), {0.,0.,0.});
    move(transformWrldToRbt({posF(0), posF(1), releasingH}), {0.,0.,0.});
    move_gripper(60);
    detachBlock("ur5", "wrist_3_link", blockName, "link");
    attachBlock("tavolo", "link", blockName, "link");
    move(transformWrldToRbt({posF(0), posF(1), workingH}), {0.,0.,0.});
    move(transformWrldToRbt(dflHndlPos), {0.,0.,0.});
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinematics");
    ros::NodeHandle nH;
    if(argc !=2){
        std::cout << "Please relaunch and insert the task number when running the node" << std::endl;
        return 1;
    }

    int task = atoi(argv[1]);

    nH.getParam("/real_robot", real_robot);
    nH.getParam("/soft_gripper", soft_gripper);
    nH.getParam("/gripper_sim", gripper_sim);

    pub = nH.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);
    attach_client = nH.serviceClient<gazebo_ros_link_attacher::Attach>("link_attacher_node/attach");
    attach_client.waitForExistence();
    detach_client = nH.serviceClient<gazebo_ros_link_attacher::Attach>("link_attacher_node/detach");
    detach_client.waitForExistence();
    
    ros::Rate loop_rate(loop_frequency);
    
    Vector6d q_des0 {-0.32, -0.78, -2.1, -1.63, -1.57,  3.49};
    
    vision::Block::ConstPtr visionNode = ros::topic::waitForMessage<vision::finBlockals>("/messmega_blocks_detectionsaggi");

    if(task == 1 || task == 2){
        for (int i=0; i< visionNode.size(); i++){ 
            blockPose releaseBlock = getBlockPose(visionNode.class_id);
            moveStrightObjFromTo({visionNode.position.x, visionNode.position.y, positionNode.position.z}, {releaseBlock.x, releaseBlock.y, releaseBlock.z}, visionNode.class_id);
            move(transformWrldToRbt(dflHndlPos), {0., 0., 0.});
        }
    }
    if(task == 3 || task == 4){
        
    }

  return 0;
}

/*
altezza blocchi = 0.863913;
*/