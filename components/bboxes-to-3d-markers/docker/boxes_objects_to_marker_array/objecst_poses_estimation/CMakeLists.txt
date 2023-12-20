cmake_minimum_required(VERSION 2.8.3)
project(objects_pose_estimation)


add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "-ffunction-sections -fdata-sections")
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--as-needed,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--as-needed,--gc-sections")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  image_transport
  roscpp  
  std_msgs
  cv_bridge
  message_generation
  sensor_msgs
  image_geometry
  object_msgs  
  tf
  depth_image_proc    
  visualization_msgs  
  vision_msgs
)

find_package(OpenCV REQUIRED COMPONENTS core imgproc)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
  /usr/include/python2.7
)

catkin_package(
  
)


add_executable(poses_estimation_node
  src/objects_pose_estimation_node.cpp
)

add_dependencies(poses_estimation_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(poses_estimation_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
