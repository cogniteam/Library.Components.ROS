FROM ros:melodic

RUN apt update && \
    apt install -y ros-melodic-audio-common && \
    rm -rf /var/lib/apt/lists/*
    
COPY /play.launch /opt/ros/melodic/share/audio_play/launch/
COPY /capture.launch /opt/ros/melodic/share/audio_capture/launch/
