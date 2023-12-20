cmake_minimum_required(VERSION 2.8.3)
project(lynx_navigation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
  dynamic_reconfigure
  angles
  geodesy
  object_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Navigation.cfg
)

add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED COMPONENTS core imgproc)

catkin_package()

###########
## Build ##
###########

include_directories(
	include
  	${catkin_INCLUDE_DIRS}
)

add_executable(lynx_navigation_node 
	src/lynx_navigation_node.cpp
	src/CostMap.cpp
	src/LaserScanDataSource.cpp
	src/SimpleTrajectoryMatcher.cpp
	src/Trajectory.cpp
	src/TrajectoryMatch.cpp
	src/TrajectorySimulator.cpp
	src/Wandering.cpp
)

add_executable(lynx_navigation_manager_node
	src/NavigationManager.cpp
	src/lynx_navigation_manager_node.cpp
) 

add_dependencies(lynx_navigation_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

add_dependencies(lynx_navigation_manager_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(lynx_navigation_node 
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)

target_link_libraries(lynx_navigation_manager_node 
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS lynx_navigation_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS lynx_navigation_manager_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)