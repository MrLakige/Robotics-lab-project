#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <iostream>
#include <fstream> 
#include <string> //#include <string.h>
#include <nlohmann/json.hpp>
#include <vector>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <gazebo_msgs/ModelStates.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <control_msgs/GripperCommandAction.h>
#include <gazebo_ros_link_attacher/SetStatic.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include "geometry_msgs/Pose.h"
#include "controller.h"
#include "../include/JointIntollerance.h"

using namespace std;

//Compensate for the interlocking height
#define INTERLOCKING_OFFSET 0.019
#define SAFE_X -0.40
#define SAFE_Y -0.13
#define SURFACE_Z  0.774

//Default position of the gripper
const double DEFAULT_POS[3] = {-0.1, -0.2, 1.2};
//Default orientation of the gripper
const Eigen::Quaterniond DEFAULT_QUAT (Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0,1,0)));
//robotics::JointIntollerance defaultPathTollerance = {"pathTollerance", 0.0, 10, //Matrix4d AH(int n, double th[6]){0.0};

typedef struct home{
    float x;
    float y;
    float z;
}homePosition, sizePosition;

typedef struct model{
    string modelName;
    homePosition homePos;
    sizePosition sizePos;
}model;

typedef struct legoModel{
    string legoName;
    geometry_msgs::Pose legoPose;
}legoModel;

model MODELS_INFO[] = {
    {"X1-Y2-Z1", {0.264589, -0.293903, 0.777}}, 
    {"X2-Y2-Z2", {0.277866, -0.724482, 0.777}}, 
    {"X1-Y3-Z2", {0.268053, -0.513924, 0.777}}, 
    {"X1-Y2-Z2", {0.429198, -0.293903, 0.777}}, 
    {"X1-Y2-Z2-CHAMFER", {0.592619, -0.293903, 0.777}}, 
    {"X1-Y4-Z2", {0.108812, -0.716057, 0.777}}, 
    {"X1-Y1-Z2", {0.088808, -0.295820, 0.777}}, 
    {"X1-Y2-Z2-TWINFILLET", {0.103547, -0.501132, 0.777}}, 
    {"X1-Y3-Z2-FILLET", {0.433739, -0.507130, 0.777}}, 
    {"X1-Y4-Z1", {0.589908, -0.501033, 0.777}}, 
    {"X2-Y2-Z2-FILLET", {0.442505, -0.727271, 0.777}}
};

/*
* Function to retrieve the center of the blocks parsing the json files 
*/
void forModels(){
    for(int i =0; i< 11;i++){
        char file[100];
        sprintf(file, "../models/%s/model.json", MODELS_INFO[i].modelName.c_str());

        ifstream myFile(file);
        if(!myFile){
            ROS_ERROR("File not found");
        }
   
        nlohmann::json data;
        myFile >> data;

        Eigen::MatrixXd corner(3,8);

        try{
            for(int i=0; i<8; i++){
                for(int j=0; j<3; j++){
                    corner(j,i) = data["corners"][i][j];
                }
            }
        }catch(const exception& e){
            cerr << "errore durante il parsing del json: "<< e.what() << endl;
            return;
        }

        float x = corner.col(0).maxCoeff() - corner.col(0).minCoeff();
        float y = corner.col(1).maxCoeff() - corner.col(1).minCoeff();
        float z = corner.col(2).maxCoeff() - corner.col(2).minCoeff();

        MODELS_INFO[i].sizePos.x = x;
        MODELS_INFO[i].sizePos.y = y;
        MODELS_INFO[i].sizePos.z = z;
    }
}

/*
* Function to retrieve the models name inside gazebo (for link attacher plugin)
*/
string getGazeboModelName(string modelName, geometry_msgs::Pose visionModelPose){
    gazebo_msgs::ModelStates::ConstPtr mdoels = ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states");
    float epsilon = 0.05;
    
    int size = sizeof(MODELS_INFO)/sizeof(MODELS_INFO[0]);

    for(size_t i=0; i<size; i++){
        if(MODELS_INFO[i].modelName.find(modelName) == string::npos){
            continue;
        }

        float ds = abs(MODELS_INFO[i].homePos.x - visionModelPose.position.x) + abs(MODELS_INFO[i].homePos.y - visionModelPose.position.y);
        if(ds <= epsilon){
            return MODELS_INFO[i].modelName;
        }
    }

    throw runtime_error("Model not found at the definede position, Error 404, Object not found !!!");
}

/*
* Function to get the position of the lego model by reading vision topic
*/
vector<legoModel> getLegosPos(bool vision=false){
    vector<legoModel> legoVector;
    //get legos position reading vision topic
    if(vision){
        gazebo_msgs::ModelStates::ConstPtr legos = ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/lego_detections");
    }else{
        gazebo_msgs::ModelStates::ConstPtr mdoels = ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states");
        gazebo_msgs::ModelStates legos = gazebo_msgs::ModelStates();

        int size = sizeof(MODELS_INFO)/sizeof(MODELS_INFO[0]);

        for(size_t i=0; i<size; i++){
            if(MODELS_INFO[i].modelName.find("X") == string::npos){
                continue;
            }
            
            legoModel tmp;
            tmp.legoName = MODELS_INFO[i].modelName;
            tmp.legoPose.position.x = MODELS_INFO[i].homePos.x;
            tmp.legoPose.position.y = MODELS_INFO[i].homePos.y;
            tmp.legoPose.position.z = MODELS_INFO[i].homePos.z;
            legoVector.push_back(tmp);
        }
    }
    return legoVector;
}

/*
* Function to send and manage the gripper opening and closing (receive the desired state of the gripper, invoce the function to change the grippers state and control if the action has succeeded)
*/
void setGripper(float value){
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> actionGripper("gripper_controller/gripper_action", true);

    // Wait for the action server to start
    if (!actionGripper.waitForServer(ros::Duration(10.0))) {
        ROS_ERROR("Action server not available.");
        return;
    }

    control_msgs::GripperCommandGoal goal;
    goal.command.position = value; // From 0.0 to 0.8
    goal.command.max_effort = -1;  // Do not limit the effort

    actionGripper.sendGoal(goal);
    // Wait for the action to finish (timeout after 10 seconds)
    bool finished = actionGripper.waitForResult(ros::Duration(10.0));

    if (finished) {
        actionlib::SimpleClientGoalState state = actionGripper.getState();
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Gripper action succeeded.");
        } else {
            ROS_ERROR("Gripper action failed.");
        }
    } else {
        ROS_ERROR("Gripper action did not complete within the specified timeout.");
        actionGripper.cancelGoal();
    }
}

/*
* Function to close the gripper
*/
void closeGripper(string gazeboModelName, float closure=0.0){
    ros::NodeHandle nH;
    ros::ServiceClient attachSrv = nH.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    setGripper(0.81-closure*10);
    ros::Duration(0.5).sleep();

    if(!gazeboModelName.empty()){
        gazebo_ros_link_attacher::Attach req;
        req.request.model_name_1 = gazeboModelName;
        req.request.link_name_1 = "link";
        req.request.model_name_2 = "robot";
        req.request.link_name_2 = "wrist_3_link";
        attachSrv.call(req);
    }
}

/*
* Function to comapre two lego models (to order the lego model)
*/
bool compareLego(const legoModel& a, const legoModel& b){
    return std::tie(a.legoPose.position.x, a.legoPose.position.y) > std::tie(b.legoPose.position.x, b.legoPose.position.y);
}

/*
* Function to open the gripper
*/
void openGripper(string gazeboModelName=NULL){
    ros::NodeHandle nH;
    ros::ServiceClient detachSrv = nH.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
    setGripper(0.0);

    if(!gazeboModelName.empty()){
        gazebo_ros_link_attacher::Attach req;
        req.request.model_name_1 = gazeboModelName;
        req.request.link_name_1 = "link";
        req.request.model_name_2 = "robot";
        req.request.link_name_2 = "wrist_3_link";
        detachSrv.call(req);
    }
}

/*
* Function to attach the block on the table nd create the first level
*/
void setModelFixed(string modelName){
    ros::NodeHandle nH;
    
    ros::ServiceClient serStaticSrv = nH.serviceClient<gazebo_ros_link_attacher::SetStatic>("/link_attacher_node/setstatic");
    ros::ServiceClient attachSrv = nH.serviceClient<gazebo_ros_link_attacher::Attach>("link_attacher_node/attach");

    gazebo_ros_link_attacher::Attach req;
    req.request.model_name_1 = modelName;
    req.request.link_name_1 = "link";
    req.request.model_name_2 = "ground_plane";
    req.request.link_name_2 = "link";
    attachSrv.call(req);

    gazebo_ros_link_attacher::SetStatic req2;
    req2.request.model_name = modelName;
    req2.request.link_name = "link";
    req2.request.set_static = true;
    serStaticSrv.call(req2);
}

/*
* Function to realigne the desired axis and angle
*/
Eigen::Quaterniond getApproachQuat(Eigen::Vector3d facingDirection, float approachAngle){
    Eigen::Quaterniond quat= DEFAULT_QUAT;
    float pitchAngle;
    float yawAngle;

    if(facingDirection == Eigen::Vector3d(0.0, 0.0, 1.0) || facingDirection == Eigen::Vector3d(0.0, 0.0, -1.0)){ //already aligned to Z axis
        pitchAngle = 0.0;
        yawAngle = 0.0;
    }else{
        if(facingDirection == Eigen::Vector3d(1.0, 0.0, 0.0) || facingDirection == Eigen::Vector3d(0.0, 1.0, 0.0)){ //not aligned to Z axis, so adjust it to be so
            pitchAngle = 0.2;
            if(abs(approachAngle)< M_PI/2){
                yawAngle = M_PI/2;
            }else{
                yawAngle = -M_PI/2;
            }
        }else{
            throw runtime_error("Invalid model state in facing direction");
        }
    }

    quat = quat*Eigen::Quaterniond( 0.0, 1.0, 0.0, pitchAngle);
    quat = quat*Eigen::Quaterniond( 0.0, 0.0, 1.0, yawAngle);
    quat = Eigen::Quaterniond( 0.0, 0.0, 1.0, approachAngle+M_PI/2)*quat;

    return quat;
}

/*
* Function to retrieve the axis that faces the camera
*/
Eigen::Vector3d getAxisFacingCamera(Eigen::Quaterniond quat){
    Eigen::Vector3d axisX =  Eigen::Vector3d(1.0, 0.0, 0.0);
    Eigen::Vector3d axisY =  Eigen::Vector3d(0.0, 1.0, 0.0);
    Eigen::Vector3d axisZ =  Eigen::Vector3d(0.0, 0.0, 1.0);

    Eigen::Vector3d newAxisX = quat*axisX;
    Eigen::Vector3d newAxisY = quat*axisY;
    Eigen::Vector3d newAxisZ = quat*axisZ;

    float angle = acos(min(max(newAxisZ.dot(axisZ), -1.0) , 1.0)); //retrieve angle between Zaxis and ZoriginalAxis

    if(angle< M_PI/3){ 
        return Eigen::Vector3d(0.0, 0.0, 1.0); //angle facing Camera
    }else{
        if(angle< M_PI/3*2*1.2){ // 60° <angle <144° 
            if(abs(newAxisX(2))>abs(newAxisY(2))){ //check which axis is nearer to Z axis 
                return Eigen::Vector3d(1.0, 0.0, 0.0);
            }else{
                return Eigen::Vector3d(0.0, 1.0, 0.0);
            }
        }else{
            return Eigen::Vector3d(0.0, 0.0, -1.0); //angle far from Z axis
        }
    }
}

/*
* Function to undersand how to move the gripper to pick the object
*/
float getApproachAngle(Eigen::Quaterniond modelQuat, Eigen::Vector3d facingDirection){
    if(facingDirection == Eigen::Vector3d(0.0, 0.0, 1.0)){ //alogned with z axis
        Eigen::Matrix3d rotationMatrix = modelQuat.toRotationMatrix();
        float yaw = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0)) - M_PI / 2;
        return yaw;
    }else{
        if(facingDirection == Eigen::Vector3d(1.0, 0.0, 0.0) || facingDirection == Eigen::Vector3d(0.0, 1.0, 0.0)){ //alogned with x or y axis
            Eigen::Vector3d axisX = Eigen::Vector3d(0.0, 1.0, 0.0);
            Eigen::Vector3d axisY = Eigen::Vector3d(-1.0, 0.0, 0.0);

            Eigen::Vector3d newAxisZ = modelQuat*Eigen::Vector3d(0.0, 0.0, 1.0);

            //scalar product to understand which angle needed to approach the object
            float dot = min(max(newAxisZ.dot(axisX), -1.0) , 1.0); 
            float det = min(max(newAxisZ.dot(axisY), -1.0) , 1.0);
            
            return atan2(det, dot);
        }else{
            if(facingDirection == Eigen::Vector3d(0.0, 0.0, -1.0)){
                Eigen::Matrix3d rotationMatrix = modelQuat.toRotationMatrix();
                float yaw = -fmod((atan2(rotationMatrix(1, 0), rotationMatrix(0, 0)) - M_PI / 2),M_PI) -M_PI;
                //float yaw = -fmod((atan2(rotationMatrix(1, 0), rotationMatrix(0, 0)) - M_PI / 2),M_PI -M_PI);
                return yaw;
            }else{
                throw runtime_error("Invalid model state in facing direction");
            }
        }
    }
}

/*
* Function to 
*/
void straighten(geometry_msgs::Pose modelPose, string gazeboModelName){
    RoboticArm roboticArm;
    float x = modelPose.position.x;
    float y = modelPose.position.y;
    float z = modelPose.position.z;
    sizePosition modelSize;

    Eigen::Quaterniond modelQuat = Eigen::Quaterniond(modelPose.orientation.x, modelPose.orientation.y, modelPose.orientation.z, modelPose.orientation.w);
    for(int i=0; i<11; i++){
        if((MODELS_INFO[i].modelName.compare(gazeboModelName))==0){
            modelSize = MODELS_INFO[i].sizePos;
        }
    }
    /*
        Calculate approach quaternion and target quaternion
    */

    Eigen::Vector3d facingDirection = getAxisFacingCamera(modelQuat);
    float approachAngle = getApproachAngle(modelQuat, facingDirection);

    ROS_INFO("Lego is facing (%f, %f, %f)", facingDirection[0], facingDirection[1], facingDirection[2]);
    ROS_INFO("Angle of approach is %f", approachAngle);

    Eigen::Quaterniond approachQuat = getApproachQuat(facingDirection, approachAngle);

    roboticArm.moveTo(x, y, std::nan(""), approachQuat);

    Eigen::Quaterniond regripQuat = DEFAULT_QUAT;
    Eigen::Quaterniond targetQuat;

    if(facingDirection == Eigen::Vector3d(1.0, 0.0, 0.0) || facingDirection == Eigen::Vector3d(0.0, 1.0, 0.0)){
        targetQuat = DEFAULT_QUAT;
        float pitchAngle = -M_PI/2 + 0.2;

        if (abs(approachAngle)< M_PI/2){
            targetQuat = targetQuat*Eigen::Quaterniond( 0.0, 0.0, 1.0, M_PI/2);
        }else{
            targetQuat = targetQuat*Eigen::Quaterniond( 0.0, 0.0, 1.0, -M_PI/2);
        }
        targetQuat = Eigen::Quaterniond( 0.0, 1.0, 0.0, pitchAngle)*targetQuat;

        if(facingDirection == Eigen::Vector3d(0.0, 1.0, 0.0)){
            regripQuat = Eigen::Quaterniond( 0.0, 0.0, 1.0, M_PI/2)*regripQuat;
        }
    }else{
        /*
            Pre-positioning
        */
        if(facingDirection == Eigen::Vector3d(0.0, 0.0, -1.0)){
            roboticArm.moveTo(std::nan(""), std::nan(""), z, approachQuat);
            closeGripper(gazeboModelName, modelSize.x); //x --> larghezza modello

            Eigen::Quaterniond tmpQuat = Eigen::Quaterniond( 0.0, 0.0, 1.0, 2*M_PI/6)*DEFAULT_QUAT;
            roboticArm.moveTo(SAFE_X, SAFE_Y, z+0.05, tmpQuat, 0.1);
            roboticArm.moveTo(std::nan(""), std::nan(""), z);
            
            openGripper(gazeboModelName);

            approachQuat = tmpQuat*Eigen::Quaterniond( 1.0, 0.0, 0.0, M_PI/2);
            targetQuat = approachQuat*Eigen::Quaterniond( 0.0, 0.0, 1.0, -M_PI);
            regripQuat = tmpQuat*Eigen::Quaterniond( 0.0, 0.0, 1.0, M_PI);
        }else{
            targetQuat = DEFAULT_QUAT;
            targetQuat = targetQuat*Eigen::Quaterniond( 0.0, 0.0, 1.0, -M_PI/2);
        }
    }
    /*
        Grip the model
    */
    float closure;
    if(facingDirection == Eigen::Vector3d(0.0, 0.0, 1.0) || facingDirection == Eigen::Vector3d(0.0, 0.0, -1.0)){
        closure = modelSize.x;
        z = SURFACE_Z + modelSize.z/2;
    }else{
        if(facingDirection == Eigen::Vector3d(1.0, 0.0, 0.0)){
            closure = modelSize.y;
            z = SURFACE_Z + modelSize.x/2;
        }else{
            if(facingDirection == Eigen::Vector3d(0.0, 1.0, 0.0)){
                closure = modelSize.x;
                z = SURFACE_Z + modelSize.y/2;
            }
        }
    }
    roboticArm.moveTo(std::nan(""), std::nan(""), z, approachQuat);
    closeGripper(gazeboModelName, closure);

    /*
        Straighten model if needed
    */
    if(facingDirection != Eigen::Vector3d(0.0, 0.0, 1.0)){
        z = SURFACE_Z + modelSize.z/2;
        roboticArm.moveTo(std::nan(""), std::nan(""), z+0.05, targetQuat, 0.1);
        roboticArm.move(0.0, 0.0, -0.05);
        openGripper(gazeboModelName);

        // Re grip the model
        roboticArm.moveTo(std::nan(""), std::nan(""), z, regripQuat, 0.1);
        closeGripper(gazeboModelName, modelSize.x);
    }
}

int main(int argc, char* argv[]){
    ROS_INFO("Initializing node of kinematics");
    ros::init(argc, argv, "sendJoints");
    
    RoboticArm roboticArm;
    forModels();

    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> actionGripper("/gripper_controller/gripper_cmd", true);
    ROS_INFO("Waiting for action of gripper controller");
    actionGripper.waitForServer();

    ros::NodeHandle nH;
    
    ros::ServiceClient serStaticSrv = nH.serviceClient<gazebo_ros_link_attacher::SetStatic>("/link_attacher_node/setstatic");
    ros::ServiceClient attachSrv = nH.serviceClient<gazebo_ros_link_attacher::Attach>("link_attacher_node/attach");
    ros::ServiceClient detachSrv = nH.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

    serStaticSrv.waitForExistence();
    attachSrv.waitForExistence();
    detachSrv.waitForExistence();

    roboticArm.moveTo(DEFAULT_POS[0], DEFAULT_POS[1], DEFAULT_POS[2], DEFAULT_QUAT, 0.0, true);

    ROS_INFO("Waiting for detection of the models");
    ros::Duration(0.5);

    vector<legoModel> legos = getLegosPos(true);
    sort(legos.begin(), legos.end(), compareLego);
    
    for(int i=0;i<legos.size();i++){
        openGripper();
        model model;
        string gazeboModelName;
        try{
            for(int j=0; j<11; j++){
                if((MODELS_INFO[j].modelName.compare(legos.at(i).legoName))==0){
                    model.modelName = MODELS_INFO[j].modelName;
                    model.homePos = MODELS_INFO[j].homePos;
                    model.sizePos = MODELS_INFO[j].sizePos;
                }
            }
        }catch(invalid_argument& e){
            ROS_ERROR("Model name was not recognized!");
            continue;
        }
        try{
            gazeboModelName = getGazeboModelName(model.modelName, legos.at(i).legoPose);
        }catch ( ros::Exception &e )
        {
            ROS_ERROR("Error occured: %s ", e.what());
            cerr << e.what() << endl;
            continue;
        }
        straighten(legos.at(i).legoPose, model.modelName);
        roboticArm.move(0, 0, 0.15);

        /*
            Go to destination
        */
        float x = model.homePos.x;
        float y = model.homePos.y;
        float z = model.homePos.z;

        z+= model.sizePos.z /2+0.004;

        ROS_INFO("Moving model {%s} to (%f, %f, %f)", model.modelName.c_str(), x, y, z);

        roboticArm.moveTo(x, y, z, DEFAULT_QUAT*Eigen::Quaterniond( 0.0, 0.0, 1.0, M_PI/2));
        // Lower the object and release
        roboticArm.moveTo(x, y, z);
        setModelFixed(gazeboModelName);
        openGripper(gazeboModelName);
        roboticArm.move(0, 0, 0.15);

        //if(roboticArm.gripperPose[0][1]> -0.3 && roboticArm.gripperPose[0][0]> 0){
        if(roboticArm.gripperPose.first[1]> -0.3 && roboticArm.gripperPose.first[0]> 0){
            roboticArm.moveTo(DEFAULT_POS[0], DEFAULT_POS[1], DEFAULT_POS[2], DEFAULT_QUAT);
        }
        for(int j=0; j<11; j++){
            if((MODELS_INFO[j].modelName.compare(legos.at(i).legoName))==0){
                MODELS_INFO[j].homePos.z += model.sizePos.z - INTERLOCKING_OFFSET;
            }
        }
    }
    
    ROS_INFO("Moving to Default Position");
    roboticArm.moveTo(DEFAULT_POS[0], DEFAULT_POS[1], DEFAULT_POS[2], DEFAULT_QUAT);
    openGripper();
    ros::Duration(0.4);

    return 0;
}
