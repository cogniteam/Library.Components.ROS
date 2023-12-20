#FROM ros:melodic
FROM nimbushaystack/haystack_image_base:latest


#COPY ./src/cliff_uv_controller haystack_ws/src/cliff_uv_controller
#COPY ./src/exploration_bridge haystack_ws/src/exploration_bridge
COPY ./src/ubiquity_magni_silver_driver haystack_ws/src/ubiquity_magni_silver_driver
COPY ./src/serial haystack_ws/src/serial



RUN apt-get update  && \  
    apt-get install ros-melodic-xacro -y && \
    apt-get install ros-melodic-rosbridge-server && \
    apt-get install ros-melodic-tf2-web-republisher && \ 
    apt-get install ros-melodic-robot-state-publisher && \
    apt-get install ros-melodic-diff-drive-controller && \
    apt-get install ros-melodic-joint-state-controller && \
    # Clear apt-cache to reduce image size
    rm /var/lib/apt/lists/* -rf

WORKDIR /haystack_ws/

RUN . /opt/ros/melodic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]


CMD ["roslaunch","magni_bringup","core.launch"]
