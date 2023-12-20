FROM ros:melodic

COPY ./map_server map_server_ws/src/map_server




RUN apt-get update && \
    apt-get install libsdl-image1.2-dev -y && \
    apt-get install libsdl-dev -y && \
    apt-get install libbullet-dev -y && \
    apt-get install libyaml-cpp-dev -y && \
    apt-get install ros-melodic-tf2 -y  && apt-get install ros-melodic-tf -y  && \
    rm /var/lib/apt/lists/* -rf
    
WORKDIR /map_server_ws/

RUN . /opt/ros/melodic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]
   
