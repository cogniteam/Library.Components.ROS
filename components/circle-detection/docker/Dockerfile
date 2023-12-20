FROM ros:melodic

COPY ./circle_detection /circle_detection_ws/src/circle_detection

WORKDIR /circle_detection_ws


RUN apt update && apt-get install ros-melodic-image-transport-plugins -y && apt-get install ros-melodic-image-transport -y && apt-get install ros-melodic-cv-bridge -y &&  rosdep update && rosdep install --from-path src --ignore-src -y && rm /var/lib/apt/lists/* -rf
RUN . /opt/ros/melodic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]


