#include <array>
#include "std_msgs/Header.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <std_msgs/Float64MultiArray.h> 
#include "sensor_msgs/JointState.h"
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include "std_msgs/String.h"
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>
#include <gazebo_ros_link_attacher/AttachRequest.h>
#include <gazebo_ros_link_attacher/AttachResponse.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <gazebo_msgs/ModelStates.h>
#include "../include/directKinematics.h"
#include "../include/inverseKinematics.h"
#include "vision/Block.h"
#include "ros_impedance_controller/generic_float.h"
#include <std_msgs/Float64.h>

using namespace std;

typedef struct model{
    string modelName;
    double  gripperOp;
    double  x;
    double  y;
}model;

model MODELS_INFO[] = {
    {"X1-Y2-Z1", 0.5, 0.264589, -0.293903}, 
    {"X2-Y2-Z2", 0.22, 0.277866, -0.724482}, 
    {"X1-Y3-Z2", 0.5, 0.268053, -0.513924}, 
    {"X1-Y2-Z2", 0.5, 0.429198, -0.293903}, 
    {"X1-Y2-Z2-CHAMFER", 0.5, 0.592619, -0.293903}, 
    {"X1-Y4-Z2", 0.5, 0.108812, -0.716057}, 
    {"X1-Y1-Z2", 0.5, 0.088808, -0.295820}, 
    {"X1-Y2-Z2-TWINFILLET", 0.5, 0.103547, -0.50113}, 
    {"X1-Y3-Z2-FILLET", 0.5, 0.433739, -0.507130}, 
    {"X1-Y4-Z1", 0.5, 0.589908, -0.501033}, 
    {"X2-Y2-Z2-FILLET", 0.22, 0.442505, -0.727271}
};

array<model, 15> detectedObjs;

Eigen::Matrix<double,6,1> current_pos = {-0.9, -2, -1.3, 0.0, 0.0, 0};

// Rotatin matrix to euler angles

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

//Euler angles to rotatin matrix

Eigen::Matrix3d eulerToRotMat(Eigen::Vector3d theta){
    Eigen::Matrix3d Rx {
                        {1,  0,  0,}, 
                        {0,  cos(theta(0)), -sin(theta(0))},
                        {0, sin(theta(0)), cos(theta(0))}
                        };
    Eigen::Matrix3d Ry {
                        {cos(theta(1)),  0,  sin(theta(1))}, 
                        {0, 1,  0},
                        {-sin(theta(1)), 0, cos(theta(1))}
                        };
    Eigen::Matrix3d Rz {
                        {cos(theta(2)),  -sin(theta(2)), 0}, 
                        {sin(theta(2)),  cos(theta(2)), 0},
                        {0, 0,  1}
                        };
    Eigen::Matrix3d rotMatrix = Rz*Ry*Rx;
    return rotMatrix;
}

void setGripper(float value){
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> actionGripper("gripper_controller/gripper_cmd", true);

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

void moveGripper(double diameter, ros::ServiceClient gripperClient){
    ros_impedance_controller::generic_float::Request req;
    ros_impedance_controller::generic_float::Response res;
    
    req.data = diameter;
    gripperClient.call(req, res);
}

Eigen::Vector3d xe(double t, Eigen::Vector3d xe0, Eigen::Vector3d xef){
    return (t*xef + (1-t)*xe0);
}

Eigen::Vector3d phie(double t, Eigen::Vector3d phie0, Eigen::Vector3d phief){
    return (t*phief + (1-t)*phie0);
}

void moveTo(Eigen::Vector3d xef, Eigen::Vector3d phief, ros::Publisher pub, double threshold){
    trajectory_msgs::JointTrajectory traj;
    //traj.header.seq;
    //traj.header.stamp;
    //traj.header.frame_id;
    traj.joint_names = {
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint"
    };

    ros::Rate loop_rate(10);

    Eigen::Matrix<double,6,1> TH0 = {-0.9, -2, -1.3, 0.0, 0.0, 0}; //SISTEMARE CON I VALORI DI FOCCHI
    directK directK = ur5DirectKinematics(TH0);  
    Eigen::Vector3d phie0 = rotMatToEuler(directK.Re).transpose();

    if(xef(1) > 0.2){
        xef(1) -= 0.01;
    }else{
        xef(1) -= 0.025;
    }

    if(xef(1) > 0.1){
        xef(0) += 0.01;
    }else{         
        if(xef(1) > -0.15){
            xef(0) += 0.005;
        }
    }

    if(xef(0) > 0.6 && xef(1) > 0.1){
        xef(0) += 0.005;
    }

    xef = xef.transpose();
    phief = phief.transpose();

    Eigen::Vector3d x = xe(1, directK.pe, xef); //Using only one point -> final position
    Eigen::Vector3d phi = phie(1, phie0, phief);

    phi = phi.transpose();

    Eigen::AngleAxisd rollAngle(phi(0), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(phi(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(phi(2), Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

    Eigen::Matrix3d rotationMatrix = q.matrix();
    Eigen::Matrix<double, 8, 6> Th= ur5inverseKinematics(x, rotationMatrix);
    cout<<"test"<<endl;  
    while (ros::ok()){
        //cout<<"move"<<endl;
        // Check if the arm is in close range, threshold of wanted position
        /*
        if ((TH0(0)>Th(6,0)-threshold and TH0(0)<Th(6,0)+threshold) and (TH0(1)>Th(6,1)-threshold and TH0(1)<Th(6,1)+threshold) and (TH0(2)>Th(6,2)-threshold and TH0(2) < Th(6,2)+threshold) and (TH0(3)>Th(6,3)-threshold and TH0(3) < Th(6,3)+threshold) and (TH0(4)>Th(6,4)-threshold and TH0(4) < Th(6,4)+threshold) and (TH0(5)>Th(6,5)-threshold and TH0(5) < Th(6,5)+threshold)){
            break;
        }

        traj.header.stamp = ros::Time::now();
        trajectory_msgs::JointTrajectoryPoint pts;

        pts.positions = {Th(6,0), Th(6,1), Th(6,2), Th(6,3), Th(6,4), Th(6,5)};
        pts.time_from_start = ros::Duration(0.2);
        //pts.accelerations={0.5, 0.5, 0.5, 0.5, 0.5, 0.5};

        cout<<pts<<endl;
        // Set the points to the trajectory
        traj.points.push_back(pts);
        // Publish the message to the topic
        pub.publish(traj);
        */

        std_msgs::Float64MultiArray msg;
        for(int i=0; i<6;i++){
            msg.data.push_back((_Float64)Th(6, i));
        }
        //msg.data = Th.col(2);
        pub.publish(msg);
    }
}

void jointState(sensor_msgs::JointState msg){
    current_pos(0) = msg.position[3];
    current_pos(1) = msg.position[2];
    current_pos(2) = msg.position[0];
    current_pos(3) = msg.position[4];
    current_pos(4) = msg.position[5];
    current_pos(5) = msg.position[6];
}

int main(int argc, char** argv){
    cout<<"Motion Planning"<<endl;
    ros::init(argc, argv, "send_joints");
    ros::NodeHandle nH;
    /*
    ros::ServiceClient attachSrv = nH.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    attachSrv.waitForExistence();
    ros::ServiceClient detachSrv = nH.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
    detachSrv.waitForExistence();
    */
    ros::ServiceClient gripperClient = nH.serviceClient<ros_impedance_controller::generic_float>("move_gripper");
    //ros::Publisher pub = nH.advertise<std_msgs::Float64MultiArray>("/trajectory_controller/command", 10);
    ros::Publisher pub = nH.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);

    ros::Subscriber sub = nH.subscribe("/joint_states", 1000, jointState);    

    //std_msgs::String yolo = *(ros::topic::waitForMessage<std_msgs::String>("/mega_blocks_detections"));
    vision::Block yolo = *(ros::topic::waitForMessage<vision::Block>("/mega_blocks_detections"));

    //cout<<yolo<<endl;
    
    /*if (yolo != "''" && !detectedObjs.empty()){
        detectedObjs.clear();
    } 
    */

    int j=0, i=0;
    //for (int i=0; i< yolo.size(); i++){
        cout<<yolo<<endl;
        detectedObjs[i].x = yolo.quatx; //DA ADATTARE AL MESSAGGIO DI MICHELE
        detectedObjs[i].y = yolo.quaty; //DA ADATTARE AL MESSAGGIO DI MICHELE
        detectedObjs[i].gripperOp = yolo.dimensionx; //DA ADATTARE AL MESSAGGIO DI MICHELE
        j++;
    //}

    cout<<"done!"<<endl;

    sleep(0.2);
    ros::Time startTime = ros::Time::now();

    double thresholdP = 0.008; //precise threshold
    double thresholdG = 0.1; //generic threshold

    for(int i=0; i<detectedObjs.size(); i++){
        double x = detectedObjs[i].x;
        double y = detectedObjs[i].y;
        moveGripper(0.0, gripperClient);

        Eigen::Vector3d xef = {x, y, 0.36};
        Eigen::Vector3d phief = {detectedObjs[i].gripperOp, M_PI, 0};

        moveTo(xef, phief, pub, thresholdG);

        xef = {x, y, 0.218};
        
        moveTo(xef, phief, pub, thresholdP);
        cout<<"done?"<<endl;

        gazebo_msgs::ModelStates ms = *(ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states"));

        int minDist = 10;
        int minIndex = 6;

        for(int k=6; k<ms.name.size(); k++){
            cout<<"ok"<<endl;
            double dist = sqrt(pow(ms.pose[i].position.x-x, 2)+ pow(ms.pose[i].position.y-y, 2));
            if (dist < minDist){
                minDist = dist;
                minIndex = i;
            }
        }
        
        ROS_INFO("Attaching gripper and lego");
        gazebo_ros_link_attacher::Attach req;
        req.request.model_name_1 = "robot";
        req.request.link_name_1 = "wrist_3_link";
        req.request.model_name_2 = ms.name[minIndex];
        req.request.link_name_2 = "link";
        //attachSrv.call(req);

        setGripper(detectedObjs[i].gripperOp);
        sleep(1);

        xef = {x, y, 0.36};
        moveTo(xef, phief, pub, thresholdG);

        double xf = detectedObjs[i].x; //nel codice prende i dati dei MODELS_INFO, in caso non funzionasse, aggiungere ciclo per ricavare indice dell'oggetto riconosciuto
        double yf = detectedObjs[i].y;
        xef = {xf, yf, 0.36};
        phief = {0, M_PI, 0};

        moveTo(xef, phief, pub, thresholdG);

        xef = {xf, yf, 0.25};
        moveTo(xef, phief, pub, thresholdP);

        ROS_INFO("Detaching gripper and lego");
        req.request.model_name_1 = "robot";
        req.request.link_name_1 = "wrist_3_link";
        req.request.model_name_2 = ms.name[minIndex];
        req.request.link_name_2 = "link";
        
        moveGripper(0.0, gripperClient);

        //detachSrv.call(req);
        sleep(2.5);

        xef = {xf, yf, 0.3};
        moveTo(xef, phief, pub, thresholdG);     
    }

    ros::Time endTime = ros::Time::now();
    ros::Duration dt = endTime-startTime;
    std::stringstream ss;
    ss << dt.sec << "." << dt.nsec;
    std::cout << ss.str() << std::endl;

    return 0;
}
