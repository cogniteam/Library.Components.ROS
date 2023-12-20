FROM ros:melodic

RUN apt update && apt-get install -y ros-melodic-ecl-threads ros-melodic-geometry-msgs && \ 
    apt-get install -y ros-melodic-nav-msgs


COPY ./yocs_velocity_smoother /yocs_velocity_smoother_ws/src/yocs_velocity_smoother

WORKDIR /yocs_velocity_smoother_ws
RUN . /opt/ros/melodic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch yocs_velocity_smoother velocity_smoother.launch