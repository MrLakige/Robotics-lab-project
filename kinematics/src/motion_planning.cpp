#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <iostream>
#include <fstream> 
#include <string> //#include <string.h>
#include <json.hpp>
#include <vector>
#include <controller_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <c/ModelStates.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "control_msgs/JointTolerance.h"
#include <control_msgs/GripperCommandAction.h>
#include <gazebo_ros_link_attacher/SetStatic.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include "geometry_msgs/Pose.h"
#include "controller.h"

using namespace std;

//Compensate for the interlocking height
#define INTERLOCKING_OFFSET = 0.019
#define SAFE_X = -0.40
#define SAFE_Y = -0.13
#define SURFACE_Z = 0.774


const double DEFAULT_POS[3] = [-0.1, -0.2, 1.2];
const Eigen::Quaterniond DEFAULT_QUAT (Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0,1,0)));
const control_msgs::JointTolerance DEFAULT_PATH_TOLERANCE;
DEFAULT_PATH_TOLERANCE->name = "path_tolerance";
DEFAULT_PATH_TOLERANCE->velocity = 10;

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
    stirng legoName;
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
}

//SE NON FUNZIONA; AGGIUNGERE BYPASS

void forModels(){
    /* for(int i =0; i< 11;i++){ 
    } */

    for(int i =0; i< 11;i++){
        string file;
        sprintf(file, "../models/%s/model.json", MODELS_INFO[i].modelName);

        ifstream myFile(file/* , std::ifstream::binary */);
        if(!myFile){
            ros::ROS_ERR("File not found");
        }
        Json::Value corners;
        myFile >> corners;

        Eigen::MatrixXd corner(3,8);

        string json_content((istreambuf_iterator<char>(myFile)), istreambuf_iterator<char>());
        try{
            nlohmann::json data = nlohmann::json::parse(json_content);
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

string getGazeboModelName(string modelName, geometry_msgs::Pose visionModelPose){
    gazebo_msgs::ModelStates::ConstPtr mdoels = ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states");
    float epsilon = 0.05;

    for(size_t i=0; i<models.name.size(); i++){
        if(models.name[i].find(modelName) == string::npos){
            continue;
        }

        float ds = abs(model_pose.position.x - vision_model_pose.position.x) + abs(model_pose.position.y - vision_model_pose.position.y);
        if(ds <= epsilon){
            return models.name[i];
        }
    }

    string msg;
    sprintf(msg, "Model %s at position %f,%f was not found!", modelName, vision_model_pose.position.x, vision_model_pose.position.y);
    throw runtime_error(msg);
}

vector<legoModel> getLegosPos(bool vision=false):
    //get legos position reading vision topic
    if(vision){
        gazebo_msgs::ModelStates::ConstPtr legos = ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/lego_detections");
    }else{
        gazebo_msgs::ModelStates::ConstPtr mdoels = ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states");
        legos = ModelStates();
        vector<legoModel> legoVector;

        for(size_t i=0; i<models.name.size(); i++){
            if(models.name[i].find("X") == string::npos){
                continue;
            }
            
            legoModel tmp;
            tmp.legoName = models.name[i];
            tmp.legoPose = models.pose[i];
            legoVector.push_back(tmp);
        }
    }
return legoVector;

void straighten(geometry_msgs::Pose modelPose, string gazeboModelName){
    RoboticArm roboticArm;
    float x = modelPose.position.x;
    float y = modelPose.position.y;
    float z = modelPose.position.z;
    Eigen::Quaterniond modelQuat = Eigen::Quaterniond(float x=modelPose.orientation.x, float y=modelPose.orientation.y, float z=modelPose.orientation.z, float w=modelPose.orientation.w);
    for(int i=0; i<11; i++){
        if(strcmp(MODELS_INFO[i].modelName,gazeboModelName)==0){
            sizePosition modelSize = MODELS_INFO[i].sizePos;
        }
    }
    """
        Calculate approach quaternion and target quaternion
    """

    Eigen::Vector3d facingDirection = getAxisFacingCamera(modelQuat);
    float approachAngle = getApproachAngle(modelQuat, facingDirection);

    ros::ROS_INFO("Lego is facing %s", facingDirection);
    ros::ROS_INFO("Angle of approach is %s", approachAngle);

    Eigen::Quaterniond approachQuat = getApproachQuat(facingDirection, approachAngle);

    roboticArm.moveTo(x, y, double z = std::nan(""), approachQuat);

    Eigen::Quaterniond regripQuat = DEFAULT_QUAT;

    if(facingDirection == Eigen::Vector3d(1.0, 0.0, 0.0) || facingDirection == Eigen::Vector3d(0.0, 1.0, 0.0)){
        Eigen::Quaterniond targetQuat = DEFAULT_QUAT;
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
        """
            Pre-positioning
        """
        if(facingDirection == Eigen::Vector3d(0.0, 0.0, -1.0)){
            roboticArm.moveTo(double x = std::nan(""), double y = std::nan(""), z, approachQuat);
            closeGripper(gazeboModelName, modelSize.sizePos.x); //x --> larghezza modello

            Eigen::Quaterniond tmpQuat = Eigen::Quaterniond( 0.0, 0.0, 1.0, 2*M_PI/6)*DEFAULT_QUAT;
            roboticArm.moveTo(SAFE_X, SAFE_Y, z+0.05, tmpQuat, 0.1);
            roboticArm.moveTo(double x = std::nan(""), double y = std::nan(""), z);
            
            openGripper(gazeboModelName);

            approachQuat = tmpQuat*Eigen::Quaterniond( 1.0, 0.0, 0.0, M_PI/2);
            targetQuat = approachQuat*Eigen::Quaterniond( 0.0, 0.0, 1.0, -M_PI);
            regripQuat = tmpQuat*Eigen::Quaterniond( 0.0, 0.0, 1.0, M_PI);
        }else{
            targetQuat = DEFAULT_QUAT;
            targetQuat = targetQuat*Eigen::Quaterniond( 0.0, 0.0, 1.0, -M_PI/2);
        }
    }
    """
        Grip the model
    """
    float closure;
    if(facingDirection == Eigen::Vector3d(0.0, 0.0, 1.0) || facingDirection == Eigen::Vector3d(0.0, 0.0, -1.0)){
        closure = modelSize.sizePos.x;
        z = SURFACE_Z + modelSize.sizePos.z/2;
    }else{
        if(facingDirection == Eigen::Vector3d(1.0, 0.0, 0.0)){
            closure = modelSize.sizePos.y;
            z = SURFACE_Z + modelSize.sizePos.x/2;
        }else{
            if(facingDirection == Eigen::Vector3d(0.0, 1.0, 0.0)){
                closure = modelSize.sizePos.x;
                z = SURFACE_Z + modelSize.sizePos.y/2;
            }
        }
    }
    roboticArm.moveTo(double x = std::nan(""), double y = std::nan(""), z, approachQuat);
    closeGripper(gazeboModelName, closure);

    """
        Straighten model if needed
    """
    if(facingDirection != Eigen::Vector3d(0.0, 0.0, 1.0)){
        z = SURFACE_Z + modelSize.sizePos.z/2;
        roboticArm.moveTo(double x = std::nan(""), double y = std::nan(""), z+0.05, targetQuat, 0.1);
        roboticArm.move(0.0, 0.0, -0.05);
        openGripper(gazeboModelName);

        // Re grip the model
        roboticArm.moveTo(double x = std::nan(""), double y = std::nan(""), z, regripQuat, 0.1);
        closeGripper(gazeboModelName, modelSize.sizePos.x);
    }
}

void closeGripper(string gazeboModelName, float closure=0.0){
    setGripper(0.81-closure*10);
    ros::Duration(0.5).sleep();

    if(!gazeboModelName.empty()){
        gazebo_ros_link_attacher::AttachRequest req = AttachRequest();
        req.model_name_1 = gazeboModelName;
        req.link_name_1 = "link";
        req.model_name_2 = "robot";
        req.link_name_2 = "wrist_3_link";
        attach_srv.call(req);
    }
}

void openGripper(string gazeboModelName=NULL){
    setGripper(0.0);

    if(!gazeboModelName.empty()){
        gazebo_ros_link_attacher::AttachRequest req = AttachRequest();
        req.model_name_1 = gazeboModelName;
        req.link_name_1 = "link";
        req.model_name_2 = "robot";
        req.link_name_2 = "wrist_3_link";
        detach_srv.call(req);
    }
}

void setModelFixed(string modelName){
    gazebo_ros_link_attacher::AttachRequest req = AttachRequest();
    req.model_name_1 = modelName;
    req.link_name_1 = "link";
    req.model_name_2 = "ground_plane";
    req.link_name_2 = "link";
    attach_srv.call(req);

    gazebo_ros_link_attacher::SetStaticRequest req = SetStaticRequest();
    req.model_name = model_name;
    req.link_name = "link";
    req.set_static = true;
    setstatic_srv.call(req);
}

Eigen::Quaterniond getApproachQuat(Eigen::Vector3d facingDirection, float approachAngle){
    Eigen::Quaterniond quat= DEFAULT_QUAT;
    float pitchAngle ;
    float yawAngle;

    if(facingDirection == Eigen::Vector3d(0.0, 0.0, 1.0) || facingDirection == Eigen::Vector3d(0.0, 0.0, -1.0)){
        pitchAngle = 0.0;
        yawAngle = 0.0;
    }else{
        if(facingDirection == Eigen::Vector3d(1.0, 0.0, 0.0) || facingDirection == Eigen::Vector3d(0.0, 1.0, 0.0)){
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
    quat = quat*Eigen::Quaterniond( 0.0, 0.0, 1.0, approachAngle+M_PI/2);

    return quat;
}

Eigen::Vector3d getAxisFacingCamera(Eigen::Quaterniond quat){
    Eigen::Vector3d axisX =  Eigen::Vector3d(1.0, 0.0, 0.0);
    Eigen::Vector3d axisY =  Eigen::Vector3d(0.0, 1.0, 0.0);
    Eigen::Vector3d axisZ =  Eigen::Vector3d(0.0, 0.0, 1.0);

    Eigen::Quaterniond newAxisX = quat*axisX;
    Eigen::Quaterniond newAxisY = quat*axisY;
    Eigen::Quaterniond newAxisZ = quat*axisZ;

    float angle = acos(min(max(newAxisZ.dot(axisZ), -1.0) , 1.0));

    if(angle< M_PI/3){
        return Eigen::Vector3d(0.0, 0.0, 1.0);
    }else{
        if(angle< M_PI/3*2*1.2){
            if(abs(newAxisX(2))>abs(newAxisY(2))){
                return Eigen::Vector3d(1.0, 0.0, 0.0);
            }else{
                return Eigen::Vector3d(0.0, 1.0, 0.0);
            }
        }else{
            return Eigen::Vector3d(0.0, 0.0, -1.0);
        }
    }
}

float getApproachAngle(Eigen::Quaterniond modelQuat, Eigen::Vector3d facingDirection){
    if(facingDirection == Eigen::Vector3d(0.0, 0.0, 1.0)){
        return modelQuat.yawPitchRoll().x() - M_PI/2;
    }else{
        if(facingDirection == Eigen::Vector3d(1.0, 0.0, 0.0) || facingDirection == Eigen::Vector3d(0.0, 1.0, 0.0)){
            Eigen::Vector3d axisX = Eigen::Vector3d(0.0, 1.0, 0.0);
            Eigen::Vector3d axisY = Eigen::Vector3d(-1.0, 0.0, 0.0);

            Eigen::Quaterniond newAxisZ = modelQuat*Eigen::Vector3d(0.0, 0.0, 1.0);

            float dot = min(max(newAxisZ.dot(axisX), -1.0) , 1.0);
            float det = min(max(newAxisZ.dot(axisY), -1.0) , 1.0);
            
            return atan2(det, dot);
        }else{
            if(facingDirection == Eigen::Vector3d(0.0, 0.0, -1.0)){
                return -(modelQuat.yawPitchRoll().x() - M_PI/2)%M_PI-M_PI;
            }else{
                throw runtime_error("Invalid model state in facing direction");
            }
        }
    }
}

void setGripper(float value){
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> action_gripper("gripper_controller/gripper_action", true);

    // Wait for the action server to start
    if (!action_gripper.waitForServer(ros::Duration(10.0))) {
        ros::ROS_ERROR("Action server not available.");
        return;
    }

    control_msgs::GripperCommandGoal goal;
    goal.command.position = value; // From 0.0 to 0.8
    goal.command.max_effort = -1;  // Do not limit the effort

    action_gripper.sendGoal(goal);
    // Wait for the action to finish (timeout after 10 seconds)
    bool finished = action_gripper.waitForResult(ros::Duration(10.0));

    if (finished) {
        actionlib::SimpleClientGoalState state = action_gripper.getState();
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ros::ROS_INFO("Gripper action succeeded.");
        } else {
            ros::ROS_ERROR("Gripper action failed.");
        }
    } else {
        ros::ROS_ERROR("Gripper action did not complete within the specified timeout.");
        action_gripper.cancelGoal();
    }
}

int main(int argc, char* argv[]){ 
    ros::ROS_INFO("Initializing node of kinematics");
    ros::init(argc, argv, "send_joints");
    
    RoboticArm roboticArm = RoboticArm(); //in caso buttare fuori roboticArm e clienti

    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> actionGripper("/gripper_controller/gripper_cmd", true);
    ros::ROS_INFO("Waiting for action of gripper controller");
    action_gripper.waitForServer(/* ros::Duration(27.0) */);

    ros::NodeHandle nH;

    ros::ServiceClient serStaticSrv = nH.advertiseService("/link_attacher_node/setstatic", SetStatic);
    ros::ServiceClient attachSrv = nH.advertiseService("/link_attacher_node/attach", Attach);
    ros::ServiceClient detachSrv = nH.advertiseService("/link_attacher_node/detach", Attach);

    serStaticSrv.waitForService("/link_attacher_node/setstatic", ros::Duration(10.0));
    attachSrv.waitForService("/link_attacher_node/attach", ros::Duration(10.0));
    detachSrv.waitForService("/link_attacher_node/detach", ros::Duration(10.0));

    roboticArm.moveTo(DEFAULT_POS[0], DEFAULT_POS[1], DEFAULT_POS[2], DEFAULT_QUAT);

    ros::ROS_INFO("Waiting for detection of the models");
    ros::Duration(0.5);

    vector<legoModel> legos = getLegosPos(true);
    sort(legos.begin(), legos.end(), [](float a, float b){return (a.second.position.x>b.second.position.x) || (a.second.position.x >= b.second.position.x)});

    for(int i=0;i<logos.size();i++){
        openGripper();
        model model;
        string gazeboModelName;
        try{
            for(int j=0; j<11; j++){
                if(strcmp(MODELS_INFO[j].modelName,legos.at(i).legoName)==0){
                    model.modelName = MODELS_INFO[j].modelName;
                    model.homePos = MODELS_INFO[j].homePos;
                    model.sizePos = MODELS_INFO[j].sizePos;
                }
            }
        }catch{
            ROS_ERR("Model name was not recognized!");
            continue
        }
        try{
            gazeboModelName = getGazeboModelName(model.modelName, legos.at(i).legoPose);
        }catch(invalid_argument& e){
            ros::ROS_ERR(e.what());
            cerr << e.what() << endl;
            continue;
        }
        straighten(legos.at(i).legoPose, model.modelName);
        roboticArm.move(0, 0, 0.15);

        """
            Go to destination
        """
        float x = model.homePos.x;
        float y = model.homePos.y;
        float z = model.homePos.z;

        z+= model.sizePos.z /2+0.004;

        ros::ROS_INFO("Moving model {%s} to {%f} {%f} {%f}", model.modelName, x, y, z);

        roboticArm.moveTo(x, y, z, DEFAULT_QUAT*Eigen::Quaterniond( 0.0, 0.0, 1.0, M_PI/2));
        // Lower the object and release
        roboticArm.moveTo(x, y, z);
        setModelFixed(gazeboModelName);
        openGripper(gazeboModelName);
        roboticArm.move(0, 0, 0.15);

        if(roboticArm.gripperPose[0][1]> -0.3 && roboticArm.gripperPose[0][0]> 0){
            roboticArm.moveTo(DEFAULT_POS[0], DEFAULT_POS[1], DEFAULT_POS[2], DEFAULT_QUAT);
        }
        for(int j=0; j<11; j++){
                if(strcmp(MODELS_INFO[j].modelName,legos.at(i).legoName)==0){
                    MODELS_INFO[j].homePos.z += model.sizePos.z - INTERLOCKING_OFFSET;
                }
        }
    }
    ros::ROS_INFO("Moving to Default Position");
    roboticArm.moveTo(DEFAULT_POS[0], DEFAULT_POS[1], DEFAULT_POS[2], DEFAULT_QUAT);
    openGripper();
    ros::Duration(0.4);
    
    return 0;
}
