FROM ros:melodic

COPY ./turtlebot_laser_navigation /turtlebot_laser_navigation_ws/src/turtlebot_laser_navigation
COPY ./waypoints_markers_publishers /turtlebot_laser_navigation_ws/src/waypoints_markers_publishers

WORKDIR /turtlebot_laser_navigation_ws
RUN . /opt/ros/melodic/setup.sh && apt update && apt install -y ros-${ROS_DISTRO}-urg* qtbase5-dev &&  rosdep update && rosdep install --from-path src --ignore-src -yr && rm /var/lib/apt/lists/* -rf
RUN . /opt/ros/melodic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch turtlebot_laser_navigation nimbus.launch
 

