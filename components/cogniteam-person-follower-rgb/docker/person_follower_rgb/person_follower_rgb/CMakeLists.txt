cmake_minimum_required(VERSION 2.8.3)
project(person_follower_rgb)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  object_msgs
  roscpp
  tf
  depth_image_proc
  image_transport
  cv_bridge
  image_geometry
  visualization_msgs
  angles
  ackermann_msgs
  vision_msgs
)

find_package(OpenCV REQUIRED)

find_package(nlohmann_json 3.2.0 REQUIRED) # for json skeletons


catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(person_follower_rgb_node 
    src/perosn_follower_rgb_node.cpp
)

add_dependencies(person_follower_rgb_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(person_follower_rgb_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(person_follower_rgb_node PRIVATE nlohmann_json::nlohmann_json)
