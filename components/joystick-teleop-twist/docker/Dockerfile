FROM ros:noetic

# COPY ./joystick_teleop /joystick_teleop_ws/src
WORKDIR /joystick_teleop_ws

RUN apt-get update && \
    apt-get install -y ros-noetic-joy ros-noetic-joy-teleop && \
    apt-get install -y ros-noetic-roslint git

RUN git clone https://github.com/ros-teleop/teleop_twist_joy.git src

RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch teleop_twist_joy teleop.launch  