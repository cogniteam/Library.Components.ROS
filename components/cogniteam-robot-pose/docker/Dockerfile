FROM ros:noetic

ENV ROS_WS=/robot_pose_ws

WORKDIR ${ROS_WS}

COPY nimbus_robot_pose ${ROS_WS}/src

RUN apt-get update -y && apt-get upgrade -y && \
    apt-get -y install ros-noetic-tf

RUN . /opt/ros/noetic/setup.sh && catkin_make

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source /robot_pose_ws/devel/setup.bash && exec $0 $@"]