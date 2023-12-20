FROM ros:noetic

COPY ./leg_tracker /leg_tracker_ws/src/leg_tracker
WORKDIR /leg_tracker_ws

RUN apt update && apt-get install ros-noetic-image-geometry -y  && apt-get install ros-noetic-tf -y &&apt-get install ros-noetic-interactive-markers -y && apt-get install ros-noetic-cv-bridge -y
RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch leg_tracker joint_leg_tracker.launch
