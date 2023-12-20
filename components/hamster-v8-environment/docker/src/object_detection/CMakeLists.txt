cmake_minimum_required(VERSION 3.0.2)
project(object_detection)

find_package(catkin REQUIRED)
catkin_package()


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  object_msgs
)

#catkin_python_setup()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
