FROM ros:kinetic

COPY ./m-explore /exploration_ws/src/m-explore
WORKDIR /exploration_ws
RUN apt-get update && rosdep update &&  \ 
	apt-get install ros-kinetic-costmap-2d -y  &&  \
	apt-get install ros-kinetic-move-base-msgs -y  &&  \  
	rosdep install  --rosdistro=kinetic --from-path src --ignore-src -yr && rm /var/lib/apt/lists/* -rf
RUN . /opt/ros/kinetic/setup.sh && catkin_make
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]


CMD roslaunch explore_lite explore.launch --screen
