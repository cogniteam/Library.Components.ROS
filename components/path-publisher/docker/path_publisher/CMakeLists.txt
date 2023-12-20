cmake_minimum_required(VERSION 2.8.3)
project(path_publisher)

find_package(catkin REQUIRED COMPONENTS
  
  geometry_msgs
  tf
  nav_msgs
  std_msgs
)
catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_definitions(-std=c++11)



add_executable(path_publisher_node 
    src/path_publisher_node.cpp
    
)
target_link_libraries(path_publisher_node
  ${catkin_LIBRARIES}

)


add_dependencies(path_publisher_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)


#############
## Install ##
#############

install(TARGETS path_publisher_node
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
