cmake_minimum_required(VERSION 2.8.3)
project(anomaly_detection_launch)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        )


catkin_package()
