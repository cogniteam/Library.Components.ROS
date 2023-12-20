cmake_minimum_required(VERSION 2.8.3)
project(lynx_localization)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  angles
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
  geodesy
  dynamic_reconfigure
  lynx_msgs
)

generate_dynamic_reconfigure_options(
  cfg/LynxLocalization.cfg
)

catkin_package()
###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lynx_gps_localization_node 
  src/lynx_gps_localization_node.cpp
  src/LynxGpsLocalization.cpp
  src/ComplimentaryFilter.cpp
  src/LynxMapGraphBuilder.cpp
)

add_executable(lynx_map_graph_builder_node 
  src/LynxMapGraphBuilder.cpp
  src/lynx_map_graph_builder.cpp
)

add_dependencies(lynx_gps_localization_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(lynx_map_graph_builder_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(lynx_gps_localization_node 
  ${PROJECT_NAME}_gencfg
)

add_dependencies(lynx_map_graph_builder_node 
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(lynx_gps_localization_node
  ${catkin_LIBRARIES}
)

target_link_libraries(lynx_map_graph_builder_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS lynx_gps_localization_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS lynx_map_graph_builder_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)