cmake_minimum_required(VERSION 3.0.2)
project(spawner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/spawner1.py
  scripts/spawner2.py
  scripts/spawner3.py
  scripts/spawner4.py
  scripts/spawner1Default.py
  scripts/spawner2Default.py
  scripts/spawner3Default.py
  scripts/spawner4Default.py
  scripts/spawnerGeneral.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
