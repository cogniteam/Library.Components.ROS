FROM ros:melodic

RUN apt update && apt install -y ros-message-generation ros-melodic-roslint ros-melodic-xacro libaria2* ros-melodic-geometry-* ros-melodic-actionlib ros-melodic-nav-msgs libaria-dev ros-melodic-tf2-eigen ros-melodic-move-base* ros-melodic-angles* ros-melodic-tf ros-melodic-tf2 ros-melodic-tf2-msgs && rm /var/lib/apt/lists/* -rf


#WORKDIR /catkin_ws/src
#RUN . /opt/ros/melodic/setup.sh && git clone https://github.com/amor-ros-pkg/rosaria.git && cd ../ && catkin_make --force-cmake

WORKDIR /catkin_ws
COPY ./Omron_LD_ROS_Package /catkin_ws/src/Omron_LD_ROS_Package
RUN . /opt/ros/melodic/setup.sh && catkin_make 
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch ros_omron_agv omron_bringup.launch
 

