FROM ros:melodic

COPY ./mir_robot /catkin_ws/src/mir_robot
WORKDIR /catkin_ws

RUN apt update && apt install -y ros-melodic-robot-state-publisher && rosdep update && rosdep install --from-path src --ignore-src -y && rm /var/lib/apt/lists/* -rf
RUN . /opt/ros/melodic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch mir_driver mir.launch
 

