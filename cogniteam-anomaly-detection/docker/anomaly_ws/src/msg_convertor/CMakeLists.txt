cmake_minimum_required(VERSION 2.8.3)
project(msg_convertor)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  topic_tools
)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(msg_convertor_node
        src/msg_convertor_node.cpp
        src/msg_convertor.cpp
)

add_dependencies(msg_convertor_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(msg_convertor_node
  ${catkin_LIBRARIES}
)

