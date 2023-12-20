cmake_minimum_required(VERSION 2.8.3)
project(person_skeleton_follower)

add_compile_options(-std=c++11)

find_package(OpenCV 3.0.0 REQUIRED )

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  image_transport
  cv_bridge
  #image_geometry
  #angles  
)

find_package(OpenCV REQUIRED)

find_package(nlohmann_json 3.2.0 REQUIRED)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Jsoncpp_INCLUDE_DIR}
)

add_executable(person_skeleton_follower_node 
    src/person_skeleton_follower_node.cpp
)

add_dependencies(person_skeleton_follower_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(person_skeleton_follower_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  
)

target_link_libraries(person_skeleton_follower_node PRIVATE nlohmann_json::nlohmann_json)

