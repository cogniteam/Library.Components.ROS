FROM ros:noetic

COPY ./joystick_teleop /joystick_teleop_ws/src
WORKDIR joystick_teleop_ws

RUN apt-get update
RUN . /opt/ros/noetic/setup.sh && apt-get install ros-noetic-joy -y && apt-get install ros-noetic-joy-teleop -y \
    && apt-get install ros-noetic-roslint -y
RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]


CMD roslaunch teleop_twist_joy teleop.launch  