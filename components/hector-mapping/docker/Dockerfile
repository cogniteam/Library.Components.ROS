FROM ros:melodic-ros-core
RUN apt-get update && apt-get install -y ros-melodic-hector-mapping && rm -rf /var/lib/apt/lists/*
COPY /hector_mapping.launch /opt/ros/melodic/share/hector_mapping/launch/
CMD ["roslaunch", "hector_mapping", "hector_mapping.launch", "map_resolution:=0.05"]