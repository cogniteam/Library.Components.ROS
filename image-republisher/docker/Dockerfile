FROM ros:melodic

COPY ./img_republisher /img_republisher_ws/src/img_republisher
WORKDIR /img_republisher_ws

RUN apt update && apt-get install ros-melodic-topic-tools -y && apt-get install ros-melodic-image-transport-plugins -y && apt-get install ros-melodic-image-transport -y && apt-get install ros-melodic-cv-bridge -y && rm /var/lib/apt/lists/* -rf
RUN . /opt/ros/melodic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch img_republisher img_republisher.launch