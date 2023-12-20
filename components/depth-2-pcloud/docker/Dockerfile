FROM ros:melodic

COPY ./depth_to_pcloud_scan /depth_to_pcloud_scan_ws/src/depth_to_pcloud_scan
WORKDIR /depth_to_pcloud_scan_ws

RUN apt update && \    
    apt-get install ros-melodic-image-transport -y && \
    apt-get install ros-melodic-pcl-ros -y && \
    apt-get install ros-melodic-pcl-msgs -y && \
    apt-get install ros-melodic-image-geometry -y  && \
    apt-get install ros-melodic-geometry-msgs -y  && \
    rosdep update && \
    rosdep install --from-path src --ignore-src -y && rm /var/lib/apt/lists/* -rf

RUN . /opt/ros/melodic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch depth_to_pcloud_scan depth_to_pcloud_scan.launch
