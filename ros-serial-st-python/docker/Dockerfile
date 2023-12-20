FROM ros:noetic

COPY ./rosserial /rosserial

RUN apt-get update && apt install ros-noetic-rosserial -y &&  rm /var/lib/apt/lists/* -rf
RUN . /opt/ros/noetic/setup.sh 

WORKDIR /rosserial/rosserial_python/launch/


CMD roslaunch ros_serial_python.launch --screen


