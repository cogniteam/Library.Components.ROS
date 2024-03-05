cmake_minimum_required(VERSION 2.8.3)
project(cogniteam_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs std_srvs
)

catkin_install_python(PROGRAMS
  scripts/action_listener_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)