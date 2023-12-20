FROM ros:noetic

ENV ROS_WS=path_publisher_ws

COPY ./path_publisher /${ROS_WS}/src/path_publisher

WORKDIR /${ROS_WS}

RUN apt-get update && \
    apt-get install ros-noetic-geometry-msgs -y && \
    apt-get install ros-noetic-nav-msgs -y && \
    apt-get install ros-noetic-sensor-msgs -y && \
    apt-get install ros-noetic-tf2-geometry-msgs -y && \
    apt-get install ros-noetic-tf2 -y  && apt-get install ros-noetic-tf -y && \
    rm /var/lib/apt/lists/* -rf

RUN . /opt/ros/noetic/setup.sh && catkin_make
	
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]