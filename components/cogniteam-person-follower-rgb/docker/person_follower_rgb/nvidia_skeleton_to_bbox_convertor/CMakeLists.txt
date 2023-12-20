cmake_minimum_required(VERSION 2.8.3)
project(nvidia_skeleton_convertor)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  object_msgs
  roscpp
  tf 
  visualization_msgs  
  ackermann_msgs
  vision_msgs
)

find_package(OpenCV  REQUIRED)

find_package(nlohmann_json 3.2.0 REQUIRED)


catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(nvidia_to_bbox_convertor_node 
    src/nvidia_to_bbox_convertor_node.cpp
)

add_dependencies(nvidia_to_bbox_convertor_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(nvidia_to_bbox_convertor_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(nvidia_to_bbox_convertor_node PRIVATE nlohmann_json::nlohmann_json)
