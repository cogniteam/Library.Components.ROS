FROM ros:noetic

COPY ./face_landsmark /face_landsmark_ws/src/face_landsmark



RUN apt update && \
    apt-get install pip -y && \
    pip install mediapipe 


WORKDIR /face_landsmark_ws



RUN apt update && apt-get install ros-noetic-cv-bridge -y  &&  \ 
    rm /var/lib/apt/lists/* -rf
RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch face_landsmark faces_landsmarks.launch
