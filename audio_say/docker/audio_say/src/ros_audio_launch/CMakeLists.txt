cmake_minimum_required(VERSION 2.8.3)

project(ros_audio_launch)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp actionlib_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

#catkin_python_setup()

generate_messages(DEPENDENCIES actionlib_msgs)

#catkin_package(CATKIN_DEPENDS message_runtime actionlib_msgs
#               INCLUDE_DIRS include)


#install(FILES
#  soundplay_node.launch
#  test.launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#install(DIRECTORY sounds
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
