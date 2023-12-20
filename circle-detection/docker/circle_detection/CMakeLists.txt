cmake_minimum_required(VERSION 2.8.3)
project(circle_detection)

add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "-ffunction-sections -fdata-sections")
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--as-needed,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--as-needed,--gc-sections")

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS  
  roscpp  
  std_msgs
  sensor_msgs
  cv_bridge  
  image_transport
  geometry_msgs

)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}

)

add_executable(circle_detection_node
  src/circle_detection/circle_detection_node.cpp
)

target_link_libraries(circle_detection_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
