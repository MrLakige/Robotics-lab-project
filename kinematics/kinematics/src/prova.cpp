//#include "gripper.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <algorithm>
#include "EigenLib.h"
#include <ros_impedance_controller/generic_float.h>

ros::ServiceClient gripperClient;

void moveGripper(int diameter){
    ros_impedance_controller::generic_float::Request req;
    ros_impedance_controller::generic_float::Response res;
    
    req.data = diameter;
    gripperClient.call(req, res);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gripperMng");
  ros::NodeHandle nH;
  gripperClient = nH.serviceClient<ros_impedance_controller::generic_float>("move_gripper");
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ROS_INFO("Gripper a 10\n");
    moveGripper(10);
    ros::spinOnce();
    ros::Duration(1.5).sleep();

    ROS_INFO("Gripper a 80\n");
    moveGripper(80);
    ros::spinOnce();
    ros::Duration(1.5).sleep();

    ROS_INFO("Gripper a 5\n");
    moveGripper(5);
    ros::spinOnce();
    ros::Duration(1.5).sleep();

    loop_rate.sleep();
  }

  return 0;
}
