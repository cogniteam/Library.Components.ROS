cmake_minimum_required(VERSION 3.0.2)
project(local_camera_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(

)

###########
## Build ##
###########


add_executable(local_camera_driver_node
  src/local_camera_driver.cpp
)


target_link_libraries(local_camera_driver_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
