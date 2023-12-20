FROM ros:noetic

COPY ./hands_pose_detectors /hands_pose_detectors_ws/src/hands_pose_detectors



RUN apt update && \
    apt-get install pip -y && \
    pip install mediapipe 


WORKDIR /hands_pose_detectors_ws



RUN apt update && apt-get install ros-noetic-cv-bridge -y  &&  \ 
    rm /var/lib/apt/lists/* -rf
RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch hands_pose_detectors hands_pose_detectors.launch
