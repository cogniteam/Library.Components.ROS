FROM ros:melodic

COPY ./hamster_driver /hamster_driver_ws/src/hamster_driver
COPY ./twist_to_ackermann /hamster_driver_ws/src/twist_to_ackermann

WORKDIR /hamster_driver_ws

RUN apt-get update && apt install ros-melodic-imu-filter-madgwick ros-melodic-serial ros-melodic-ackermann-msgs ros-melodic-ackermann-steering-controller ros-melodic-pcl-ros ros-melodic-angles -y && rosdep update &&  rosdep install  --rosdistro=melodic --from-path src --ignore-src -yr && rm /var/lib/apt/lists/* -rf
RUN . /opt/ros/melodic/setup.sh && catkin_make
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

ENV HAMSTER_ID agent1

CMD roslaunch hamster_driver hamster_driver.launch --screen