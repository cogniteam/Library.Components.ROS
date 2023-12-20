FROM ros:noetic

COPY ./ros_rtsp /ros_rstp_ws/src/
WORKDIR ros_rstp_ws

RUN apt-get update && sudo apt-get install python3
RUN apt-get -y install python3-pip -y
RUN apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio -y
RUN apt-get install libfcl-dev -y
RUN apt-get install libgstrtspserver-1.0-dev gstreamer1.0-rtsp -y
RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD ["roslaunch", "ros_rtsp", "rtsp_streams.launch"]
