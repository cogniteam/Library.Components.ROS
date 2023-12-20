FROM ros:melodic

COPY ./rr_bosch_bno055_imu /bosch_bno055_imu_ws/src/rr_bosch_bno055_imu


WORKDIR /bosch_bno055_imu_ws

RUN apt-get update && apt-get install python-serial && apt-get install python-yaml && apt-get install ros-melodic-tf -y && rosdep update && rosdep install  --rosdistro=melodic --from-path src --ignore-src -yr && rm /var/lib/apt/lists/* -rf
RUN . /opt/ros/melodic/setup.sh && catkin_make
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD ["roslaunch", "rr_bosch_bno055_imu", "example.launch"]