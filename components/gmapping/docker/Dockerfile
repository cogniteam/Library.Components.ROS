FROM ros:melodic-ros-core
RUN apt-get update && apt-get install -y ros-melodic-gmapping ros-melodic-gmapping-dbgsym && rm -rf /var/lib/apt/lists/*
COPY /slam_gmapping.launch  /opt/ros/melodic/share/gmapping/
CMD ["roslaunch", "gmapping", "slam_gmapping.launch"]
