#FROM ros:melodic
FROM cognimbus/kobuki_driver

RUN apt update

RUN apt install ros-melodic-dwa-local-planner -y && \
    apt install ros-melodic-teb-local-planner -y && \
    apt install ros-melodic-clear-costmap-recovery -y && \
    apt install ros-melodic-move-base-msgs -y && \
    apt install ros-melodic-navfn -y && \
    apt install ros-melodic-rotate-recovery -y && \
    rm /var/lib/apt/lists/* -rf

WORKDIR /
COPY turtlebot_navigation /turtlebot_navigation_ws/src/turtlebot_navigation
COPY move_base /turtlebot_navigation_ws/src/move_base

WORKDIR turtlebot_navigation_ws


RUN . /opt/ros/melodic/setup.sh  && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch turtlebot_navigation move_base.launch.xml