FROM ros:melodic

COPY ./velodyne_ros /velodyne_lidar_ws/src/vlp16
WORKDIR /velodyne_lidar_ws

RUN apt update && apt-get install ros-melodic-pcl-ros -y && \
    apt-get install ros-melodic-pcl-msgs -y && \
    apt-get install libyaml-cpp-dev -y &&\
    apt-get install libpcap-dev -y &&\
    apt-get install ros-melodic-diagnostic-updater &&\
    apt-get install ros-melodic-roslint &&\
    apt-get install ros-melodic-angles 

RUN . /opt/ros/melodic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch velodyne_pointcloud VLP16_points.launch --screen

