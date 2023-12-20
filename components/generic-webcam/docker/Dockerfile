FROM ros:melodic-ros-core

RUN apt-get update && apt-get install -y \
 ros-melodic-usb-cam ros-melodic-image-transport-plugins && rm -rf /var/lib/apt/lists/*
 
COPY ./usb_cam_clear.launch /usb_cam_clear.launch


CMD roslaunch usb_cam_clear.launch
