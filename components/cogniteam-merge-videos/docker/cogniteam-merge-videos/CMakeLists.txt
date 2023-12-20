cmake_minimum_required(VERSION 2.8.3)
project(merge_videos)

add_compile_options(-std=c++14)

find_package(OpenCV REQUIRED )

find_package(catkin REQUIRED COMPONENTS
  
  image_transport  
  cv_bridge
  roscpp  
  image_geometry
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
  dynamic_reconfigure

)

generate_dynamic_reconfigure_options(
  cfg/Video.cfg
)



catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp std_msgs  
)

###########
## Build ##
###########



include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(merge_videos_node 
    src/merge_videos_node.cpp

)

add_dependencies(merge_videos_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(merge_videos_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
