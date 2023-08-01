#include <cmath>
#include <algorithm>
#include "ros/ros.h"
#include "Eigen/Core"
#include "kinematics.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Pose.h"

class RoboticArm {
  public:
    double gripperState=0;
    const std::string& controllerTopic="/trajectory_controller";
    trajectory_msgs::JointTrajectory default_joint_trajectory = trajectory_msgs.msg.JointTrajectory();

    RoboticArm(gripperState, controllerTopic):
    gripperState(gripperState)
    controllerTopic(controllerTopic)
    {
        jointsNames = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ];

        default_joint_trajectory.joint_names = jointsNames;
        control_msgs::JointTrajectoryControllerState JointState = getControllerState(controllerTopic).actual.positions;
        std::vector<double> Pose = kinematics.getPose(JointState);
        gripperPose = {Pose[0], Pose[1], Pose[2], Quaternion(Matrix=Pose[3])};
        joinsPub = nh.advertise<trajectory_msgs::JointTrajectorys>(controllerTopic+"/command", 10);
    }

    void move(double dx=0, double dy=0, double dz=0, const Quaternion& deltaQuat=Quaternion(1, 0, 0, 0), bool blocking=true){
        Eigen::Vector3d startingPose = gripperPose.first;
        Quaternion startingQuaternion = gripperPose.second;
        double tx, ty, tz;
        tx = startingPose[0] + dx;
        ty = startingPose[1] + dy;
        tz = startingPose[2] + dz;
        Quaternion targetQuat = startingQuaternion* deltaQuat;
        
        moveTo(tx, ty, tz, targetQuat, blocking);
    }

    void moveTo(double x = std::nan(""), double y = std::nan(""), double z = std::nan(""), const Quaternion& targetQuat = Quaternion(1, 0, 0, 0), double zRaise = 0.0, bool blocking = true){
        
        auto smooth = [](double percentValue, double period=M_PI){
            return (1 - std::cos(percentValue * period)) / 2;
        };

        Eigen::Vector3d startingPose = gripperPose.first;
        Quaternion startingQuaternion = gripperPose.second;

        if (std::isnan(x)){
            x = startingPose[0];
        }

        if (std::isnan(y)){
            y = startingPose[1];
        }

        if (std::isnan(z)){
            z = startingPose[2];
        }

        double dx, dy, dz;
        dx = x - startingPose[0];
        dy = y - startingPose[1];
        dz = z - startingPose[2];

        double lenght = std::sqrt(dx*dx + dy*dy + dz*dz)*300 + 80;
        double speed = lenght;

        int steps = static_cast<int>(lenght);
        double step = 1/steps;

        for(double i = 0; i<= 1.0+step, i+=step){
            double i2 = smooth(i, 2*M_PI);
            double i1 = smooth(i);

            Quaternion grip = Quaternion::slerp(startingQuaternion, targetQuat, i1);
            sendJoints(startingPose[0] + i1*dx, startingPose[1]+i1*dy, startingPose[2]+i1*dz + i2*zRaise, grip, 1.0/speed*0.9);
            ros::Duration(1.0/speed).sleep();
        }
        if(blocking){
            waitForPosition(0.005, 0.08);
        }

        gripperPose = (x, y, z), targetQuat;
    }

    void sendJoints(double x, double y, double z, const Quaternion& quat, double duration=1.0){
        std::vector<double> jointState = kinematics.getJoints(x, y, z, quat.toRotationMatix());
        
        trajectory_msgs::JointTrajectory trajectory = default_joint_trajectory;

        for(int i = 0; i<2; i++){
            trajectory_msgs::JointTrajectoryPoint points;
            points.positions = jointStates;
            points.velocities = [0, 0, 0, 0, 0, 0];
            points.time_from_start = ros::Duration(duration);

            trajectory.points = {points};
            jointsPub.publish(trajectory);
        }
    }

    void waitForPosition(double  tollerancePos=0.01, double tolleranceVel=0.01, double timeout=2.0){
        ros::Time endTime = ros::Time::now() + ros::Duration(timeout);
        while(ros::Time::now()< endTime){
            control_msgs::JointTrajectoryControllerState msg = getControllerState(controllerTopic, 10);
            double v = std::accumulate(msg.actual.velocities.begin(), msg.actual.velocities.end(), 0.0);
            if( v < tolleranceVel){
                for(size_t i = 0; i< msg.actual.positions.size(); i++){
                    if(std::abs(msg.actual.positions[i] - msg.desired.positions[i])> tollerancePos){
                        break;
                    }
                    return;
                }
            }
        }
        ros::ROS_WARN("Timeout waiting for position")
    }

    private:

    double gripperState;
    std::string controllerTopic;
    std::vector<std::string> jointsNames;
    trajectory_msgs::JointTrajectory default_joint_trajectory;
    std::pair<Eigen::Vector3d, Quaternion> gripperPose;
    ros::NodeHandle nh;
    ros::Publisher joinsPub;

    control_msgs::JointTrajectoryControllerState getControllerState(const std::string& controllerTopic, double timeout=0){
        return ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>(controllerTopic + "/state", ros::Duration(timeout));
    }
};
