cmake_minimum_required(VERSION 2.8.3)
project(lynx_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  sensor_msgs
)

add_message_files(
  FILES
  StateDrive.msg
  StateImu.msg
  Path.msg
  PathEdge.msg
  Waypoint.msg
  Mission.msg
  ModuleState.msg
  States.msg
  Encoder.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs nav_msgs std_msgs sensor_msgs
)

catkin_package()
