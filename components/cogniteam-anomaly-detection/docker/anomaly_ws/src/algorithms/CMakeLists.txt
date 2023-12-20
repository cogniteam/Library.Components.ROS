cmake_minimum_required(VERSION 2.8.3)
project(algorithms)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        )


catkin_package()

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(algorithms_node
        src/algorithms_node.cpp
        src/algorithms.cpp
        src/SimpleAnomalyDetector.cpp
        src/ZScoreAnomalyDetector.cpp
        src/HybridAnomalyDetector.cpp
        src/minCircle.cpp
        include/timeseries.h
        include/AnomalyReport.h
        )

add_dependencies(algorithms_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(algorithms_node
        ${catkin_LIBRARIES}
        )
