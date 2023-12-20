FROM ros:melodic-ros-core
COPY ./add_two_ints /nimbus_ws/src/add_two_ints
WORKDIR /nimbus_ws
RUN apt-get update && apt-get install build-essential -y ros-melodic-roscpp-tutorials && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/melodic/setup.sh && catkin_make
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch beginner_tutorials client.launch --screen
