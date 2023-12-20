FROM ros:noetic
RUN apt-get update && apt-get install -y ros-noetic-audio-common && \
    apt-get update && apt-get install -y ros-noetic-usb-cam && \
    apt-get update && apt-get install -y ros-noetic-image-transport-plugins
COPY ./usb_cam_clear.launch /usb_cam_clear.launch


CMD roslaunch usb_cam_clear.launch
