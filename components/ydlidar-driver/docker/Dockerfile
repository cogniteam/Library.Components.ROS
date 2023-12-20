FROM ros:noetic

RUN apt update && apt-get update

RUN apt install -y git

WORKDIR /YDLIDAR

# Install SDK
RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git && \
    mkdir YDLidar-SDK/build && cd YDLidar-SDK/build && cmake .. && make && make install

# Install ROS driver
RUN git clone https://github.com/YDLIDAR/ydlidar_ros_driver.git ydlidar_ws/src/ydlidar_ros_driver && \
    . /opt/ros/noetic/setup.sh && cd ydlidar_ws && catkin_make

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source /YDLIDAR/ydlidar_ws/devel/setup.bash && exec $0 $@"]