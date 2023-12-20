cmake_minimum_required(VERSION 2.8.3)
project(img_republisher)

add_compile_options(-std=c++11)

find_package(OpenCV 3.0.0 REQUIRED )

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

# add_executable(img_republisher_node 
#     src/img_republisher_node.cpp
# )

# add_dependencies(img_republisher_node
#     ${${PROJECT_NAME}_EXPORTED_TARGETS} 
#     ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(img_republisher_node
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )
