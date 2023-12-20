FROM ros:noetic

COPY ./rplidar_ros /rplidar_ros_ws/src/rplidar_ros


WORKDIR /rplidar_ros_ws

RUN apt-get update && rm -rf /var/lib/apt/lists/* && rm -rf /opt/ros/noetic/share/rplidar_ros/launch/rplidar.launch


RUN . /opt/ros/noetic/setup.sh && catkin_make
	
	
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

 




