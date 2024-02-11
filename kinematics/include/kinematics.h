#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "typedefs.h"
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>
#include "vision/Block.h"

static bool real_robot = false;
static bool soft_gripper = false;
static bool gripper_sim = false;

static ros::Publisher pub;
static ros::ServiceClient gripperClient;
static std_msgs::Float64MultiArray jointState_msg_sim;
static std_msgs::Float64MultiArray jointState_msg_robot;

static ros::ServiceClient attach_client;
static ros::ServiceClient detach_client;

static double loop_time = 0.;
static double loop_frequency = 250.;

geometry_msgs::Pose getPose(double x, double y, double z);

//visionSim rotationTest{getPose(0.367994, 0.598631, 0.917500), 1, 1.565439, 0.000051, 1.569999, "X1-Y4-Z2"}; // X
//visionSim rotationTest{getPose(0.367994, 0.598631, 0.917500), 1, 1.565439, 0.000051, -1.569999, "X1-Y2-Z2"}; // -X
//visionSim rotationTest{getPose(0.367994, 0.598631, 0.917500), 1, 1.568446, -0.000086, -3.141591, "X1-Y3-Z2"}; // y
//visionSim rotationTest{getPose(0.367994, 0.598631, 0.917500), 1, -1.568446, -0.000086, 3.141591, "X1-Y4-Z1"}; //-Y

static visionSim initialBlockInfo{getPose(0.280577, 0.691699, 0.863913), 4, 0, 0, 0, "X1-Y2-Z2"};

static visionSim initialBlockInfo4[4]{
    {getPose(0.470980, 0.725608, 0.863913), 4, 0, 0, 0, "X1-Y2-Z2"},
    {getPose(0.355449, 0.708060, 0.863913), 4, 0, 0, 0, "X1-Y1-Z2"}, 
    {getPose(0.367994, 0.598631, 0.900000), 4, -1.565924, 0., 3.141590, "X1-Y3-Z2"} 
    //{getPose(0.414135, 0.725608, 0.863913), 4, 0, 0, 0, "X1-Y4-Z2"} 
};

static visionSim initialBlockInfo3[4]{
    {getPose(0.414135, 0.725608, 0.870002), 4, 0, 0, 0, "X1-Y4-Z2"}, 
    {getPose(0.367994, 0.598631, 0.900000), 4, 1.565439, 0.000051, -1.569999, "X1-Y3-Z2-FILLET"}, 
    {getPose(0.123776, 0.672390, 0.870002), 4, 0, 0, 0, "X1-Y2-Z1"}, 
    {getPose(0.280577, 0.691699, 0.870002), 4, 0, 0, 0, "X1-Y1-Z2"}
};

static visionSim initialBlockInfo2[10]{
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

/**
 * @brief  Function to send the desired joints configuration.
 *
 * @param joint_pos Vector x6 with the robot arm joints configutation.
 * @param gripper_pos Vector x3 with the gripper joints configutation.
 */
void sendDesJstate(const Vector6d & joint_pos, const Eigen::Vector3d & gripper_pos);

/**
 * @brief  Function to call the serice to open/close the real gripper.
 *
 * @param diameter diametre of the gripper opening.
 */
void moveGripper(const double diameter);

/**
 * @brief  Function to map the diametre to the gripper goints to make the movement smooth.
 *
 * @param diameter diametre of the gripper opening.
 * @return result of the function computation.
 */
double mapToGripperJoints(double diameter);

/**
 * @brief  Function to set the robotic arm joints in the struct to take trace of them.
 *
 * @param q Vector x6 with the robotic arm joints configuration.
 */
void setJoints(Vector6d q);

/**
 * @brief  Function to set the gripper joints in the struct to take trace of them.
 *
 * @param q Vector x3 with the gripper joints configuration.
 */
void setGripper(Eigen::Vector3d gripper);

/**
 * @brief  Function to set the roboti arm and gripper joints in the struct to take trace of them.
 *
 * @param q Vector x6 with the joints configuration.
 */
void setJointsAndGripper(Vector6d q, Eigen::Vector3d gripper);

/**
 * @brief  Function to set the gripper joints after computing the mapToGripperJoints func.
 *
 * @param diameter diametre of the gripper opening.
 */
void setNewGripperPosition(double diameter);

/**
 * @brief  Function to call the attach service.
 *
 * @param model1 first model name.
 * @param link1 first link name.
 * @param model2 second model name.
 * @param link2 second link name.
 */
void attachBlock(const char* model1, const char* link1, const char* model2, const char* link2);

/**
 * @brief  Function to call the detach service.
 *
 * @param model1 first model name.
 * @param link1 first link name.
 * @param model2 second model name.
 * @param link2 second link name.
 */
void detachBlock(const char* model1, const char* link1, const char* model2, const char* link2);

/**
 * @brief  Function to open the gripper after all computations.
 *
 * @param diameter diametre of the gripper opening.
 */
void move_gripper(double diameter);

/**
 * @brief  Function to read the joint configuration from the robot and save them in the function to keep trace of them.
 */
void readJoints();

/**
 * @brief  Function to move the robotic arm.
 *
 * @param poseF desired end effector position.
 * @param orientF desired end effector orientation.
 */
void move(Eigen::Vector3d poseF, Eigen::Vector3d orientF);

/**
 * @brief  Function to transform the coordinates from the world frame to the robot frame.
 *
 * @param p position of the end effector in the world frame.
 * @return position of the end effector in the robot frame after the computation.
 */
Eigen::Vector3d transformWrldToRbt(Eigen::Vector3d p);

/**
 * @brief  Function to transform the coordinates from the robot frame  to the world frame.
 *
 * @param p position of the end effector in the robot frame.
 * @return position of the end effector in the world frame after the computation.
 */
Eigen::Vector3d transformRbtToWrld(Eigen::Vector3d p);

/**
 * @brief  Function to get several values for each block for different computations.
 *
 * @param blockName the name of the block of the required data.
 * @return blockInfo a struct with the needed data.
 */
blockInfo getBlockPose(std::string blockName);

/**
 * @brief  Function to get the index of the required block to create the castle based on the layer number.
 *
 * @param visionNode simualated message from the vision node.
 * @param num number of the layer.
 * @return the index value
 */
int getBlockIndexFromArrayDefault(visionSim* visionNode, int num);

/**
 * @brief  Function to get the index of the required block to create the castle based on the layer number.
 *
 * @param visionNode message from the vision node.
 * @param num number of the layer.
 * @return the index value
 */
int getBlockIndexFromArray(vision::Block::ConstPtr visionNode, int num);

/**
 * @brief  Function to move an object from one point to another one .
 *
 * @param posI initial position of the block.
 * @param posF release positions of the block.
 * @param blockName name of the handled block.
 * @param gripperClosure gripper closure.
 */
void moveP2PObject(Eigen::Vector3d posI, Eigen::Vector3d posF, const char* blockName, double gripperClosure);

/**
 * @brief  Function to set the block straight based on the position of the block.
 *
 * @param pos Position of the block.
 * @param rot Rotation  of the block.
 * @param block block struct with the required data  of the handled block.
 */
void setBlockUp(Eigen::Vector3d pos, Eigen::Vector3d rot, blockInfo block);

/**
 * @brief  Function to move the block to construct the castle.
 *
 * @param posI initial position of the block.
 * @param posF release positions of the block.
 * @param blockName name of the handled block.
 * @param gripperClosure gripper closure.
 */
void moveForCastleConstruction(Eigen::Vector3d posI, Eigen::Vector3d posF, const char* blockName, double gripperClosure);

/**
 * @brief  Function to move a standing/straight block to a atrget position.
 *
 * @param posI initial position of the block.
 * @param blockName block struct with the required data of the handled block.
 * @param posF release positions of the block.
 */
void moveStraightObjToTargetPos(Eigen::Vector3d posI, blockInfo blockName, Eigen::Vector3d posF);


#endif
