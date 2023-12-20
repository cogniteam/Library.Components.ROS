FROM ros:kinetic

COPY ./lizi /lizi_ws/src/lizi
WORKDIR /lizi_ws/src/lizi


RUN apt update && cd lizi && apt install wget -y &&./setup.sh


WORKDIR /lizi_ws/

RUN . /opt/ros/kinetic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

# CMD roslaunch kobuki_launch kobuki.launch serialport:=/dev/ttyUSB0 base:=kobuki kobuki_publish_tf:=true
 

