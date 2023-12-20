FROM ros:noetic

WORKDIR /ros_rstp_ws/src

RUN apt-get update && sudo apt-get -y install python3 git && \
    apt-get install -y python3-pip -y && \
    apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio -y && \
    apt-get install -y libfcl-dev && \
    apt-get install -y libgstrtspserver-1.0-dev gstreamer1.0-rtsp 

RUN mkdir src && cd src && git clone https://github.com/CircusMonkey/ros_rtsp.git

RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD ["roslaunch", "ros_rtsp", "rtsp_streams.launch"]
