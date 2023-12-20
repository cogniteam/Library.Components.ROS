FROM ros:noetic

ENV HANDS_POSE_WS=hands_pose_detectors_ws

# copy package content
COPY ./hands_pose_detectors /${HANDS_POSE_WS}/src/hands_pose_detectors

# install packages
RUN apt update && \
    apt-get install pip -y && \
    pip install mediapipe && \
    apt update && apt-get install ros-noetic-cv-bridge -y  &&  \ 
    rm /var/lib/apt/lists/* -rf

WORKDIR /${HANDS_POSE_WS}

# RUN apt update && apt-get install ros-noetic-cv-bridge -y  &&  \ 
#     rm /var/lib/apt/lists/* -rf
RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch hands_pose_detectors hands_pose_detectors.launch
