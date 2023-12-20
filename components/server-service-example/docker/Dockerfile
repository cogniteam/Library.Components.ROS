FROM ros:melodic-ros-core
RUN apt-get update && apt-get install -y ros-melodic-roscpp-tutorials && rm -rf /var/lib/apt/lists/*
COPY /server.launch /opt/ros/melodic/share/roscpp_tutorials/launch/
CMD ["roslaunch","roscpp_tutorials", "server.launch"]
