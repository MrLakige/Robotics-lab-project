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
#include "geometry_msgs/Pose.h"
#include "vision/Block.h"

geometry_msgs::Pose getPose(double x, double y, double z){
    geometry_msgs::Pose pose;

    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = 0.;
    pose.orientation.y = 0.;
    pose.orientation.z = 0.;
    pose.orientation.w = 0.;

    return pose;
}

//visionSim rotationTest{getPose(0.367994, 0.598631, 0.917500), 1, 1.565439, 0.000051, -1.569999, "X1-Y3-Z2"};
//visionSim rotationTest{getPose(0.367994, 0.598631, 0.917500), 1, 1.565439, 0.000051, 1.569999, "X1-Y3-Z2"};
//visionSim rotationTest{getPose(0.367994, 0.598631, 0.917500), 1, -1.565439, 0.000051, 0.000051, "X1-Y3-Z2"};
visionSim rotationTest{getPose(0.367994, 0.598631, 0.917500), 1, -1.565439, 0.000051, 0.000051, "X1-Y2-Z2"};

visionSim initialBlockInfo{getPose(0.280577, 0.691699, 0.863913), 4, 0, 0, 0, "X1-Y2-Z2"};

visionSim initialBlockInfo4[4]{
    {getPose(0.280577, 0.691699, 0.863913), 4, 0, 0, 0, "X1-Y2-Z2"},
    {getPose(0.123776, 0.672390, 0.863913), 4, 0, 0, 0, "X1-Y1-Z2"}, 
    {getPose(0.367994, 0.598631, 0.900000), 4, -1.565924, 0., 3.141590, "X1-Y3-Z2"}, 
    {getPose(0.414135, 0.725608, 0.863913), 4, 0, 0, 0, "X1-Y4-Z2"} 
};

visionSim initialBlockInfo3[4]{
    {getPose(0.414135, 0.725608, 0.863913), 4, 0, 0, 0, "X1-Y4-Z2"}, 
    {getPose(0.367994, 0.598631, 0.900000), 4, 0, 1.570791, 3.141593, "X2-Y2-Z2"}, 
    {getPose(0.123776, 0.672390, 0.863913), 4, 0, 0, 0, "X1-Y2-Z1"}, 
    {getPose(0.280577, 0.691699, 0.863913), 4, 0, 0, 0, "X1-Y1-Z2"}
};

visionSim initialBlockInfo2[11]{
    /* {getPose(0.433096, 0.730949, 0.863913), 11, 0, 0, 0, "X1-Y4-Z1"},
    {getPose(0.348063, 0.733234, 0.863913), 11, 0, 0, 0, "X1-Y2-Z1"},
    {getPose(0.225137, 0.722685, 0.863913), 11, 0, 0, 0, "X1-Y2-Z2"},
    {getPose(0.152209, 0.700987, 0.863913), 11, 0, 0, 0, "X1-Y3-Z2"},
    {getPose(0.278950, 0.649104, 0.863913), 11, 0, 0, 0, "X1-Y4-Z2"},
    {getPose(0.349409, 0.590814, 0.863913), 11, 0, 0, 0, "X2-Y2-Z2"},
    {getPose(0.418686, 0.541031, 0.863913), 11, 0, 0, 0, "X1-Y1-Z2"},
    {getPose(0.340979, 0.413728, 0.863913), 11, 0, 0, 0, "X1-Y2-Z2-CHAMFER"},
    {getPose(0.146508, 0.387718, 0.863913), 11, 0, 0, 0, "X1-Y2-Z2-TWINFILLET"},
    {getPose(0.164513, 0.495294, 0.863913), 11, 0, 0, 0, "X2-Y2-Z2-FILLET"},
    {getPose(0.180767, 0.536327, 0.863913), 11, 0, 0, 0, "X1-Y3-Z2-FILLET"} */
    {getPose(0.891367, 0.692272, 0.863913), 11, 0, 0, 0, "X1-Y4-Z1"},
    {getPose(0.233934, 0.734162, 0.863913), 11, 0, 0, 0, "X1-Y2-Z1"},
    {getPose(0.520516, 0.722642, 0.863913), 11, 0, 0, 0, "X1-Y2-Z2"},
    {getPose(0.329110, 0.700797, 0.863913), 11, 0, 0, 0, "X1-Y3-Z2"},
    {getPose(0.619331, 0.656876, 0.863913), 11, 0, 0, 0, "X1-Y4-Z2"},
    {getPose(0.789534, 0.590818, 0.863913), 11, 0, 0, 0, "X2-Y2-Z2"},
    {getPose(0.924661, 0.541030, 0.863913), 11, 0, 0, 0, "X1-Y1-Z2"},
    {getPose(0.179915, 0.577316, 0.863913), 11, 0, 0, 0, "X1-Y2-Z2-TWINFILLET"},
    {getPose(0.753272, 0.736383, 0.863913), 11, 0, 0, 0, "X2-Y2-Z2-FILLET"},
    {getPose(0.431801, 0.723457, 0.863913), 11, 0, 0, 0, "X1-Y3-Z2-FILLET"}
};

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

blockInfo getBlockPose(std::string blockName){
    blockInfo block;

    if(strcmp(blockName.c_str(),"X1-Y1-Z2")==0){
        block.x = 0.849936;
        block.y = 0.541031;
        block.z = 0.863913;
        block.gripperClosure = 50;
        block.zOffset = .0;
        block.graspingH = 1.02;
        //block.standingGraspingH = ;
        block.yOffset = 0.017;
        block.classId = "X1-Y1-Z2";
    }
    if(strcmp(blockName.c_str(),"X1-Y2-Z1")==0){
        block.x = 0.779313;
        block.y = 0.733234;
        block.z = 0.863913;
        block.gripperClosure = 50;
        block.zOffset = .03;
        block.graspingH = 1.02;
        block.classId = "X1-Y2-Z1";
    }
    if(strcmp(blockName.c_str(),"X1-Y2-Z2")==0){
        block.x = 0.656387;
        block.y = 0.722685;
        block.z = 0.863913;
        block.gripperClosure = 50;
        block.zOffset = .03;
        block.graspingH = 1.02;
        block.standingGraspingH = 1.05;
        block.classId = "X1-Y2-Z2";
    }     
    if(strcmp(blockName.c_str(),"X1-Y2-Z2-CHAMFER")==0){
        block.x = 0.772229;
        block.y = 0.413728;
        block.z = 0.863913;
        block.gripperClosure = 50;
        block.zOffset = .03;
        block.graspingH = 1.02;
        block.classId = "X1-Y2-Z2-CHAMFER";
    }       
    if(strcmp(blockName.c_str(),"X1-Y2-Z2-TWINFILLET")==0){
        block.x = 0.577758;
        block.y = 0.387718;
        block.z = 0.863913;
        block.gripperClosure = 50;
        block.zOffset = .03;
        block.graspingH = 1.02;
        block.classId = "X1-Y2-Z2-TWINFILLET";
    }       
    if(strcmp(blockName.c_str(),"X1-Y3-Z2")==0){
        block.x = 0.583459;
        block.y = 0.700987;
        block.z = 0.863913;
        block.gripperClosure = 50;
        block.zOffset = .06;
        block.graspingH = 1.02;
        block.yOffset = 0.15;
        block.classId = "X1-Y3-Z2";
    }
    if(strcmp(blockName.c_str(),"X1-Y3-Z2-FILLET")==0){
        block.x = 0.612017;
        block.y = 0.536327;
        block.z = 0.863913;
        block.gripperClosure = 50;
        block.zOffset = .06;
        block.graspingH = 1.02;
        block.classId = "X1-Y3-Z2-FILLET";
    }
    if(strcmp(blockName.c_str(),"X1-Y4-Z1")==0){
        block.x = 0.864346;
        block.y = 0.730949;
        block.z = 0.863913;
        block.gripperClosure = 50;
        block.zOffset = .09;
        block.graspingH = 1.02;
        block.yOffset = 0.15;
        block.classId = "X1-Y4-Z1";
    }
    if(strcmp(blockName.c_str(),"X1-Y4-Z2")==0){
        block.x = 0.7102;
        block.y = 0.649104;
        block.z = 0.863913;
        block.gripperClosure = 50;
        block.zOffset = .09;
        block.graspingH = 1.02;
        block.yOffset = 0.15;
        block.classId = "X1-Y4-Z2";
    }
    if(strcmp(blockName.c_str(),"X2-Y2-Z2")==0){
        block.x = 0.780659;
        block.y = 0.590814;
        block.z = 0.863913;
        block.gripperClosure = 80;
        block.zOffset = .03;
        block.graspingH = 1.04;
        block.classId = "X2-Y2-Z2";
    }
    if(strcmp(blockName.c_str(),"X2-Y2-Z2-FILLET")==0){
        block.x = 0.595763;
        block.y = 0.495294;
        block.z = 0.863913;
        block.gripperClosure =  80;
        block.zOffset = .03;
        block.graspingH = 1.04;
        block.classId = "X2-Y2-Z2-FILLET";
    }
    return block;
}

int getBlockIndexFromArray(visionSim* visionNode, int num){
    int k=0;
    for(int i=0; i<4; i++){
        if(strcmp(visionNode[i].class_id.c_str(), castleBlocksOrder[num])==0){
            k=i;
            return k;
        }
    }
    return k;
}

void moveP2PObject(Eigen::Vector3d posI, Eigen::Vector3d posF, const char* blockName, double gripperClosure){
    move(transformWrldToRbt({posI(0), posI(1), workingH}), {0.,0.,0.});
    move_gripper(80); 
    move(transformWrldToRbt({posI(0), posI(1), posI(2)/* 1.1 */}), {0.,0.,0.}); //CAMBIARE ALTEZZA DI GRASPING IN BASE AL TIPO DI OGGETTO
    attachBlock("ur5", "wrist_3_link", blockName, "link");
    move_gripper(gripperClosure);
    move(transformWrldToRbt({posI(0), posI(1), workingH}), {0.,0.,0.});
    move(transformWrldToRbt(transitionalPos), {0.,0.,0.});
    move(transformWrldToRbt({posF(0), posF(1), workingH}), {0.,0.,0.});
    move(transformWrldToRbt({posF(0), posF(1),  posI(2)/* 1.1 */}), {0.,0.,0.});
    detachBlock("ur5", "wrist_3_link", blockName, "link");
    move_gripper(80);
    move(transformWrldToRbt({posF(0), posF(1), workingH}), {0.,0.,0.});
}

//void setBlockUp(Eigen::Vector3d pos, Eigen::Vector3d rot, int gripperClosure, double zOffset, const char * classId, double graspingH, double yOffset){
void setBlockUp(Eigen::Vector3d pos, Eigen::Vector3d rot, blockInfo block){
    double x = pos(0);
    double y = pos(1);
    double z = pos(2);
    double roll = rot(0);
    double pitch = rot(1);
    double yaw = rot(2);

    if((roll>=-1.60 && roll<=-1.50 && yaw>=3.1 && yaw<=3.2) || (roll>=1.50 && roll<=1.60 && yaw>=-0.1 && yaw<=0.1)){//lying on short side facing -y azis
        move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1), 1.27}), {0.,0.,0.});
        move(transformWrldToRbt({x, y-0.027, 1.27}), {0.,0.,0.});
        move(transformWrldToRbt({x, y-0.027, 1.1}), {0.,0.,0.});
        move_gripper(80);
        move(transformWrldToRbt({x, y-.027, block.standingGraspingH}), {0., 0., 0.});
        attachBlock("ur5", "wrist_3_link", block.classId, "link");
        move_gripper(block.gripperClosure);
        move(transformWrldToRbt({x, y-.027, 1.27}), {0., 0., 0.});
        move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1)+0.13, 1.13}), {0.,0.,0.}); //sistemare hrot (1.13) in base all'oggetto
        move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1)+0.13, 1.13}), {0., M_PI_2, M_PI_2});
        move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1)+0.13, 0.93}), {0., M_PI_2, M_PI_2});
        detachBlock("ur5", "wrist_3_link", block.classId, "link");
        move_gripper(80);
        move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1)+0.13, 1.13}), {0., M_PI_2, M_PI_2});
        move(transformWrldToRbt(dflHndlPos), {0.,0.,0.});
    }else{
        if((roll>=-1.60 && roll<=-1.5 && yaw>=-1.60 && yaw<=-1.5) || (roll>=1.50 && roll<=1.60 && yaw>=1.50 && yaw<=1.60)){//lying on short side facing x azis
            move(transformWrldToRbt(dflHndlPos), {0.,0.,0.});
            moveP2PObject({x+.027, y, z}, {0.4053226, 0.707422, 1.27}, block.classId, block.gripperClosure);
            move(transformWrldToRbt(dflHndlPos), {0.,0.,0.});
            move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1), 1.13}), {-M_PI_2,0.,M_PI_2});
            move(transformWrldToRbt({dflHndlPos(0)+0.070, dflHndlPos(1), 0.91}), {-M_PI_2,0.,M_PI_2});
            move_gripper(80);
            move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1), 0.91}), {-M_PI_2,0.,M_PI_2});
            //move_gripper(60);
            attachBlock("ur5", "wrist_3_link", block.classId, "link");
            move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1), workingH}), {-M_PI_2,0.,M_PI_2});
            //move(transformWrldToRbt(transitionalPos), {0.,0.,0.});
            move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1), workingH}), {0.,0.,0.});
            move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1), releasingH}), {0.,0.,0.});
            detachBlock("ur5", "wrist_3_link", block.classId, "link");
            move_gripper(80);
            attachBlock("tavolo", "link", block.classId, "link");
            move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1), workingH}), {0.,0.,0.});
            move(transformWrldToRbt(dflHndlPos), {0.,0.,0.});
        }else{
            if((roll>=-1.60 && roll<=-1.5 && yaw>=1.50 && yaw<=1.60) || (roll>=1.50 && roll<=1.60 && yaw>=-1.60 && yaw<=-1.5)){//lying on short side facing -x azis
                move(transformWrldToRbt(dflHndlPos), {0.,0.,0.});
                move(transformWrldToRbt({x+.13, y, 1.13}), {-M_PI_2,0.,M_PI_2});
                move(transformWrldToRbt({x+.13, y, 0.93}), {-M_PI_2,0.,M_PI_2});
                move_gripper(80);
                move(transformWrldToRbt({x+.11, y, 0.93}), {-M_PI_2,0.,M_PI_2});
                attachBlock("ur5", "wrist_3_link", block.classId, "link");
                move(transformWrldToRbt(dflHndlPos), {0.,0.,0.});
                move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1)+0.13, 1.13}), {0., M_PI_2, M_PI_2});
                move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1)+0.13, 0.93}), {0., M_PI_2, M_PI_2});
                detachBlock("ur5", "wrist_3_link", block.classId, "link"); //GLITCH THAT MOVES THE OBJECT TOT THE RIGHT PLACE BY A BIT (OTHERWISE MOVE IT A BIT FORMARD)
                move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1)+0.13, 1.13}), {0., M_PI_2, M_PI_2});
                move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1)-.027, 1.1}), {0., 0., 0.});
                move_gripper(80);
                move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1)-.027, 1.06}), {0., 0., 0.});
                attachBlock("ur5", "wrist_3_link", block.classId, "link");
                move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1)-.027, 1.27}), {0., 0., 0.});
                move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1)+0.13, 1.13}), {0.,0.,0.}); //sistemare hrot (1.13) in base all'oggetto
                move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1)+0.13, 1.13}), {0., M_PI_2, M_PI_2});
                move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1)+0.13, 0.913}), {0., M_PI_2, M_PI_2}); //sistemare altezza release in caso, altrimenti prendere subito dopo il blocco, in modo da poi attaccarloa l tavolo prima che ruoti
                detachBlock("ur5", "wrist_3_link", block.classId, "link");
                move_gripper(80);
                move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1)+0.13, 1.13}), {0., M_PI_2, M_PI_2});
                move(transformWrldToRbt(dflHndlPos), {0.,0.,0.});
            }else{
                if((roll>=-1.60 && roll<=-1.5 && (yaw>=-0.27 && yaw<=0.27 || yaw>=6.15 && yaw<=6.4)) || (roll>=1.50 && roll<=1.60 && yaw>=3.1 && yaw<=3.2)){//lying on short side facing y azis
                    move(transformWrldToRbt(dflHndlPos), {0.,0.,0.});
                    moveP2PObject({x, y+.017, z}, {dflHndlPos2}, block.classId, block.gripperClosure);
                    move(transformWrldToRbt(dflHndlPos), {0.,0.,0.});
                    move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1)+.045, 1.13}), {0., M_PI_2, M_PI_2});
                    move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1)+.045, 0.92}), {0., M_PI_2, M_PI_2});
                    move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1)+.03, 0.92}), {0., M_PI_2, M_PI_2});
                    attachBlock("ur5", "wrist_3_link", block.classId, "link");
                    move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1)+.03, workingH}), {0., M_PI_2, M_PI_2});
                    move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1), workingH}), {0.,0.,0.});
                    move(transformWrldToRbt({dflHndlPos(0), dflHndlPos(1), 1.0027}), {0.,0.,0.});
                    detachBlock("ur5", "wrist_3_link", block.classId, "link");
                    move(transformWrldToRbt(dflHndlPos), {0.,0.,0.});
                }else{//standing along y axis
                    moveP2PObject({x, y, z}, {dflHndlPos(0), dflHndlPos(1), dflHndlPos(2)}, block.classId, block.gripperClosure);
                    std::cout << "block already straight" << std::endl;
                }
            }
        }
    }

}

void moveForCastleConstruction(Eigen::Vector3d posI, Eigen::Vector3d posF, const char* blockName){
    move(transformWrldToRbt({posI(0), posI(1), workingH}), {0.,0.,0.});
    move_gripper(80); 
    move(transformWrldToRbt({posI(0), posI(1), 1.1}), {0.,0.,0.}); //CAMBIARE ALTEZZA DI GRASPING IN BASE AL TIPO DI OGGETTO
    attachBlock("ur5", "wrist_3_link", blockName, "link");
    //move_gripper(60);
    move(transformWrldToRbt({posI(0), posI(1), workingH}), {0.,0.,0.});
    move(transformWrldToRbt(transitionalPos), {0.,0.,0.});
    move(transformWrldToRbt({posF(0), posF(1), workingH}), {0.,0.,0.});
    move(transformWrldToRbt({posF(0), posF(1),  posF(2)}), {0.,0.,0.});
    detachBlock("ur5", "wrist_3_link", blockName, "link");
    move_gripper(80);
    attachBlock("tavolo", "link", blockName, "link");
    move(transformWrldToRbt({posF(0), posF(1), restingH}), {0.,0.,0.});
}

void moveStraightObjToTargetPos(Eigen::Vector3d posI, blockInfo blockName, Eigen::Vector3d posF){
    move(transformWrldToRbt({posI(0), posI(1), workingH}), {0.,0.,0.});
    move_gripper(80); 
    move(transformWrldToRbt({posI(0), posI(1), blockName.graspingH}), {0.,0.,0.});
    attachBlock("ur5", "wrist_3_link", blockName.classId, "link");
    move_gripper(blockName.gripperClosure);
    move(transformWrldToRbt({posI(0), posI(1), workingH}), {0.,0.,0.});
    move(transformWrldToRbt(transitionalPos), {0.,0.,0.});
    move(transformWrldToRbt({posF(0), posF(1), workingH}), {0.,0.,0.});
    move(transformWrldToRbt({posF(0), posF(1), blockName.graspingH}), {0.,0.,0.});
    detachBlock("ur5", "wrist_3_link", blockName.classId, "link");
    move_gripper(80);
    attachBlock("tavolo", "link", blockName.classId, "link");
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
    
    vision::Block::ConstPtr visionNode = ros::topic::waitForMessage<vision::Block>("/mega_blocks_detections");

    if(task == 1){
        for (int i=0; i< visionNode->size; i++){ 
            blockInfo releaseBlock = getBlockPose(visionNode->class_id[i].c_str());
            moveStraightObjToTargetPos({visionNode->pose[i].position.x, visionNode->pose[i].position.y, visionNode->pose[i].position.z}, releaseBlock, {releaseBlock.x, releaseBlock.y, releaseBlock.graspingH});
            move(transformWrldToRbt(dflHndlPos), {0., 0., 0.});
        }    
    }

     if(task == 2){
        Eigen::Vector3d tmp = {0.914134, 0.318471, 1.02};
        for (int i=0; i< visionNode->size; i++){ 
            blockInfo releaseBlock = getBlockPose(visionNode->class_id[i].c_str());
            moveStraightObjToTargetPos({visionNode->pose[i].position.x, visionNode->pose[i].position.y, visionNode->pose[i].position.z}, releaseBlock, {tmp(0), tmp(1), releaseBlock.graspingH});
            tmp = {visionNode->pose[i].position.x, visionNode->pose[i].position.y, releaseBlock.graspingH};
        }    
    }
    /*
    if(task == 2){
        visionSim* visionNode = initialBlockInfo2;
        Eigen::Vector3d tmp = {0.914134, 0.318471, 1.02};
        for (int i=0; i<10; i++){           
            blockInfo releaseBlock = getBlockPose(visionNode->class_id[i].c_str());
            moveStraightObjToTargetPos({visionNode->pose[i].position.x, visionNode->pose[i].position.y, visionNode->pose[i].position.z}, releaseBlock, {tmp(0), tmp(1), releaseBlock.graspingH});
            tmp = {visionNode->pose[i].position.x, visionNode->pose[i].position.y, releaseBlock.graspingH};
        }        
    }
    if(task == 3){
        visionSim* visionNode = initialBlockInfo3;
        for(int i=0; i<visionNode->size; i++){
            blockInfo releaseBlock = getBlockPose(visionNode->class_id[i]);
            //setBlockUp({visionNode[i].pose.position.x, visionNode[i].pose.position.y, visionNode[i].pose.position.z},{visionNode[i].rotx, visionNode[i].roty, visionNode[i].rotz}, releaseBlock.gripperClosure, releaseBlock.zOffset, visionNode[i].class_id.c_str(), releaseBlock.graspingH, releaseBlock.yOffset);
            setBlockUp({visionNode->pose[i].position.x, visionNode->pose[i].position.y, visionNode->.pose[i].position.z},{visionNode->rotx[i], visionNode->roty[i], visionNode->rotz[i]}, releaseBlock);
            moveStraightObjToTargetPos({dflHndlPos(0), dflHndlPos(1), 1.13}, releaseBlock, {releaseBlock.x, releaseBlock.y, releaseBlock.z});
        }
    }
    if(task == 4){
        visionSim* visionNode = initialBlockInfo4;
        for(int k=0; k<4; k++){
            int i = getBlockIndexFromArray(visionNode, k);
            double z = 1.02+(k*0.06);
            blockInfo releaseBlock = getBlockPose(visionNode->class_id[i]);
            //setBlockUp({visionNode[i].pose.position.x, visionNode[i].pose.position.y, visionNode[i].pose.position.z},{visionNode[i].rotx, visionNode[i].roty, visionNode[i].rotz}, releaseBlock.gripperClosure, releaseBlock.zOffset, visionNode[i].class_id.c_str(), releaseBlock.graspingH, releaseBlock.yOffset);
            setBlockUp({visionNode->pose[i].position.x, visionNode->pose[i].position.y, visionNode->pose[i].position.z},{visionNode->rotx[i], visionNode->roty[i], visionNode->rotz[i]}, releaseBlock);
            moveForCastleConstruction(dflHndlPos, {castleBuildingPoint(0), castleBuildingPoint(1), z}, visionNode->class_id[i].c_str());
            //moveStraightObjToTargetPos({dflHndlPos(0), dflHndlPos(1), 1.13}, releaseBlock);
        }
        move(transformWrldToRbt(dflHndlPos), {0.,0.,0.});
    }   
    */
  return 0;
}