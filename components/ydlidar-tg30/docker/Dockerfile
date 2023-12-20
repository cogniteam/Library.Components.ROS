FROM ros:melodic

COPY ./ydlidar_ros_ws /ydlidar_ros_ws
WORKDIR /ydlidar_ros_ws

# RUN apt-get update &&  rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/melodic/setup.sh && catkin_make
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD ["roslaunch", "ydlidar_ros", "TG.launch"]
