FROM ros:noetic

RUN apt update && \
    apt install -y ros-noetic-audio-common && \
    apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev -y && \
    apt-get install python3-pip -y && \
    python3 -m pip install pyyaml && \
    rm -rf /var/lib/apt/lists/*
    
COPY ./audio_say/src audio_say/src

WORKDIR /audio_say/

RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch ros_audio_launch say_string.launch
