cmake_minimum_required(VERSION 2.8.3)
project(hamster_wandering)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
)

find_package(OpenCV REQUIRED )

catkin_package()

###########
## Build ##
###########

include_directories(
	include
  	${catkin_INCLUDE_DIRS}
)

add_executable(hamster_wandering_node 
	src/hamster_wandering_node.cpp
	src/CostMap.cpp
	src/LaserScanDataSource.cpp
	src/SimpleTrajectoryMatcher.cpp
	src/Trajectory.cpp
	src/TrajectoryMatch.cpp
	src/TrajectorySimulator.cpp
	src/Wandering.cpp
)

target_link_libraries(hamster_wandering_node 
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS hamster_wandering_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz/
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
)
