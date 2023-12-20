FROM ros:noetic

COPY ./src /laser_filters_ws/src/

WORKDIR /laser_filters_ws

RUN apt update && rosdep update
# RUN apt install ros-noetic-image-geometry -y && apt install ros-noetic-image-transport-plugins -y && apt install ros-noetic-tf2-geometry-msgs -y && apt install ros-noetic-geometry -y
RUN rosdep install -i --from-path src --ignore-src -y
RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

