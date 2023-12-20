FROM ros:noetic

COPY ./nextVision_cam_ws /nextVision_cam_ws/src/next_vision_cam

WORKDIR /nextVision_cam_ws

RUN apt -y update && apt -y upgrade

RUN apt-get -y install python3-pip
RUN pip3 install pymavlink
RUN . /opt/ros/noetic/setup.sh && catkin_make
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]
