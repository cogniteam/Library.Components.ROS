FROM ros:noetic

   
ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=waffle_pi

RUN apt update && \
    apt-get install keyboard-configuration -y && \
    apt install ros-noetic-dynamixel-sdk -y && \ 
    apt install ros-noetic-turtlebot3-msgs -y && \ 
    apt install ros-noetic-turtlebot3 -y  && \ 
    apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy -y && \ 
    apt-get install ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc -y  && \
    apt-get install ros-noetic-rosserial-arduino -y && \ 
    apt-get install ros-noetic-rosserial-python ros-noetic-rosserial-client -y && \ 
    apt-get install ros-noetic-rosserial-msgs && \ 
    apt-get install ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro -y && \ 
    apt-get install ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz -y && \ 
    apt-get install ros-noetic-interactive-markers -y && \
    apt-get install ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server -y && \
    apt-get install ros-noetic-gmapping ros-noetic-navigation -y
    
COPY /turtlebot3_core.launch /opt/ros/noetic/share/turtlebot3_bringup/launch/
COPY /turtlebot3_lidar.launch /opt/ros/noetic/share/turtlebot3_bringup/launch/
COPY /turtlebot3_rpicamera.launch /opt/ros/noetic/share/turtlebot3_bringup/launch/
COPY /move_base.launch /opt/ros/noetic/share/turtlebot3_navigation/launch/
COPY /turtlebot3_gmapping.launch /opt/ros/noetic/share/turtlebot3_slam/launch/
