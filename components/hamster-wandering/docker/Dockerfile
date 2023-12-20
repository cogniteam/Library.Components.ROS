FROM ros:noetic

COPY ./hamster_wandering /hamster_wandering_ws/src/hamster_wandering

WORKDIR /hamster_wandering_ws

RUN apt-get update && \
    apt-get install ros-noetic-ackermann-msgs && \
    rosdep update && \
    rosdep install  --rosdistro=noetic --from-path src --ignore-src -yr && rm /var/lib/apt/lists/* -rf

RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD ["roslaunch", "hamster_wandering", "wandering.launch"]