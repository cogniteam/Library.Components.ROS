FROM ros:melodic

COPY ./amcl amcl_ws/src/amcl


COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

RUN apt-get update && \
    apt-get install ros-melodic-geometry-msgs -y && \
    apt-get install ros-melodic-diagnostic-updater -y && \
    apt-get install ros-melodic-dynamic-reconfigure -y && \
    apt-get install ros-melodic-nav-msgs -y && \
    apt-get install ros-melodic-sensor-msgs -y && \
    apt-get install ros-melodic-std-srvs -y && \
    apt-get install ros-melodic-map-server -y && \
    apt-get install ros-melodic-tf2-geometry-msgs -y && \
    apt-get install ros-melodic-tf2 -y  && apt-get install ros-melodic-tf -y && \
    rm /var/lib/apt/lists/* -rf
    
WORKDIR /amcl_ws/

RUN . /opt/ros/melodic/setup.sh && catkin_make

CMD  roslaunch amcl cogniteam_amcl.launch --screen

   
