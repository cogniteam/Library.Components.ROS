

# sudo apt-get install ros-melodic-serial
# sudo apt-get install ros-melodic-pcl-ros


cmake_minimum_required(VERSION 2.8.3)
project(hamster_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS cmake_modules angles)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  serial
  tf
  pcl_ros
)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hamster_driver_node 
    src/hamster_driver_node.cpp
    src/WirelessInfo.cpp
    src/calibration/AutoImuCalibrator.cpp
    src/calibration/RobotStateEstimator.cpp
)

target_link_libraries(hamster_driver_node ${catkin_LIBRARIES})


add_executable(hamster_imu_calibration_node 
    src/hamster_imu_calibration_node.cpp
    src/calibration/HardIronCalibration.cpp
    src/calibration/SoftIronCalibration.cpp
)

target_link_libraries(hamster_imu_calibration_node ${catkin_LIBRARIES})


add_executable(hamster_imu_covariance_calibration_node 
    src/hamster_imu_covariance_calibration_node.cpp
)

target_link_libraries(hamster_imu_covariance_calibration_node ${catkin_LIBRARIES})


if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(calibration_tests 
    	test/calibration_tests.cpp
    	src/calibration/HardIronCalibration.cpp
    	src/calibration/SoftIronCalibration.cpp
	)
    
    target_link_libraries(calibration_tests 
    	${catkin_LIBRARIES}
    )
    
endif()

#############
## Install ##
#############

install(TARGETS hamster_driver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
