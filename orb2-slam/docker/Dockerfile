FROM ros:melodic-robot
RUN apt-get update && apt-get install -y ros-melodic-cv-bridge ros-melodic-image-transport  ros-melodic-tf2-geometry-msgs && rm -rf /var/lib/apt/lists/*

WORKDIR /
RUN mkdir orb_ws
WORKDIR /orb_ws
RUN mkdir src
WORKDIR /orb_ws/src/
RUN git clone https://github.com/appliedAI-Initiative/orb_slam_2_ros.git
WORKDIR /orb_ws/
RUN . /opt/ros/melodic/setup.sh && \
    catkin_make

WORKDIR /
COPY ./orb_slam2_d435_rgbd.launch /orb_ws/src/orb_slam_2_ros/ros/launch/
COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh
CMD /entrypoint.sh 
