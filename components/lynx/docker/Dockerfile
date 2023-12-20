FROM ros:melodic

COPY ./lynx /lynx_ws/src/lynx
COPY ./twist_to_ackermann /lynx_ws/src/twist_to_ackermann

WORKDIR /lynx_ws

RUN apt-get update && apt install ros-melodic-serial ros-melodic-ackermann-msgs ros-melodic-ackermann-steering-controller ros-melodic-pcl-ros ros-melodic-angles -y && rosdep update &&  rosdep install  --rosdistro=melodic --from-path src --ignore-src -yr && rm /var/lib/apt/lists/* -rf
RUN . /opt/ros/melodic/setup.sh && catkin_make
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

