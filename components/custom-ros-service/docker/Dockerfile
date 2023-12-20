FROM ros:noetic

COPY ./src /custom_service_ws/src

WORKDIR /custom_service_ws


RUN apt update && \
    apt-get install pip -y && \
    pip install names
    
RUN rosdep update
RUN DEBIAN_FRONTEND=noninteractive rosdep install --from-paths src --ignore-src --rosdistro=noetic -r -y
RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]
