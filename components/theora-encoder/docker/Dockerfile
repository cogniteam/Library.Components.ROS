FROM ros:melodic

COPY ./theora_info /theora_ws/src/theora_info
WORKDIR /theora_ws

RUN apt update && apt-get install ros-melodic-image-transport-plugins -y && rm /var/lib/apt/lists/* -rf
RUN . /opt/ros/melodic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch theora_info theora_info_publisher.launch
