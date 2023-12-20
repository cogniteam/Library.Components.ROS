FROM ros:noetic

COPY ./map_saver_ws/ map_saver_ws/




RUN apt-get update && \
    apt-get install libsdl-image1.2-dev -y && \
    apt-get install libsdl-dev -y && \
    apt-get install libbullet-dev -y && \
    apt-get install libyaml-cpp-dev -y && \
    apt-get install ros-noetic-tf2 -y  && apt-get install ros-noetic-tf -y  && \
    apt-get install ros-noetic-map-server -y && \
    rm /var/lib/apt/lists/* -rf
    
WORKDIR /map_saver_ws

RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]
   
