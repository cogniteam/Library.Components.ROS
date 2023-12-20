FROM ros:noetic

RUN apt update && \
    apt install -y git-core && \
    apt-get install -y ros-noetic-rviz && \
    apt-get install -y qt5-default && \
    rm -rf /var/lib/apt/lists/*
    
RUN git clone -b noetic https://github.com/ccny-ros-pkg/imu_tools.git imu_tools_ws/src

WORKDIR /imu_tools_ws/

RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch imu_filter_madgwick imu_filter_madgwick.launch
