cmake_minimum_required(VERSION 2.8.3)
project(depth_to_pcloud_scan)

add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "-ffunction-sections -fdata-sections")
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--as-needed,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--as-needed,--gc-sections")

find_package(OpenCV 3.0.0 REQUIRED )

find_package(catkin REQUIRED COMPONENTS
  
  
  pcl_ros
  geometry_msgs
  image_transport
  roscpp  
  std_msgs
  cv_bridge
  message_generation
  sensor_msgs
  image_geometry
  tf
  depth_image_proc    

)


catkin_package()


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}

)


# dpeth_to_grayscale

add_executable(depth_to_pcloud_scan_node
  src/depth_to_pcloud_scan/depth_to_pcloud_scan_node.cpp
)


target_link_libraries(depth_to_pcloud_scan_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
