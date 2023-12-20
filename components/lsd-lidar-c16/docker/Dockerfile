FROM ros:melodic

COPY ./lslidar_c16 /lsd_lidar_ws/src/LSC16
WORKDIR /lsd_lidar_ws

RUN apt update && apt-get install ros-melodic-pcl-ros -y && \
    apt-get install ros-melodic-pcl-msgs -y && \
    apt-get install ros-melodic-angles && \
    apt-get install ros-melodic-cv-bridge -y && \
    apt-get install libyaml-cpp-dev -y && \
    apt-get install ros-melodic-diagnostic-updater && \
    apt-get install libpcap-dev -y


RUN . /opt/ros/melodic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch lslidar_c16_decoder lslidar_c16.launch --screen

