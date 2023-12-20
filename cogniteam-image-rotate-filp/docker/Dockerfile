FROM ros:melodic

COPY ./cogniteam-image-rotate-filp /image_rotate_filp_ws/src/

WORKDIR /image_rotate_filp_ws

WORKDIR /image_rotate_filp_ws

RUN apt update && apt-get install ros-melodic-image-transport-plugins -y && \
    apt-get install ros-melodic-image-transport -y && \
    apt-get install ros-melodic-cv-bridge -y && \ 
    rosdep update && rosdep install --from-path src --ignore-src -y && rm /var/lib/apt/lists/* -rf
RUN . /opt/ros/melodic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch image_rotate_filp image_rotate_filp.launch
