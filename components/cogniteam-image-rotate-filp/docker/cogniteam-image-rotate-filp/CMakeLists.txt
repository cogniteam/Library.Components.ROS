cmake_minimum_required(VERSION 2.8.3)
project(image_rotate_filp)

add_compile_options(-std=c++11)

find_package(OpenCV  REQUIRED )

find_package(catkin REQUIRED COMPONENTS
  
  roscpp  
  image_transport
  cv_bridge
  
)

find_package(OpenCV REQUIRED)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_rotate_filp_node 
    src/image_rotate_filp_node.cpp
)

add_dependencies(image_rotate_filp_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(image_rotate_filp_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
