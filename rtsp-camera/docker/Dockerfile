FROM ros:melodic

RUN apt update && apt-get install ros-melodic-video-stream-opencv -y && rm /var/lib/apt/lists/* -rf

COPY /camera.launch /opt/ros/melodic/share/video_stream_opencv/launch/
