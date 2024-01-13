cmake_minimum_required(VERSION 3.0.2)
project(kinematics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  control_msgs
  actionlib
  message_generation
)

add_message_files(
    FILES
    JointIntollerance.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(${PROJECT_NAME} src/directKinematics.cpp src/functions.cpp src/inverseDifferentialKinematics.cpp src/inverseKinematics.cpp src/kinematics.cpp )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

