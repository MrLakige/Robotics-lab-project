#include "gripper.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gripperMng");
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ROS_INFO("Gripper a 10\n");
    move_gripper(10);
    ros::spinOnce();
    ros::Duration(1.5).sleep();

    ROS_INFO("Gripper a 80\n");
    move_gripper(80);
    ros::spinOnce();
    ros::Duration(1.5).sleep();

    ROS_INFO("Gripper a 5\n");
    move_gripper(5);
    ros::spinOnce();
    ros::Duration(1.5).sleep();

    loop_rate.sleep();
  }


  return 0;
}
