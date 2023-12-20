FROM ros:melodic-ros-core
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt -q -qq update && \
    DEBIAN_FRONTEND=noninteractive apt -y install ros-melodic-realsense2-camera ros-melodic-image-transport-plugins && \
    rm -rf /var/lib/apt/lists/*


COPY rs_camera_with_jpeg_level.launch /home/rs_camera_with_jpeg_level.launch
WORKDIR /home
RUN . /opt/ros/melodic/setup.sh

CMD roslaunch rs_camera_with_jpeg_level.launch enable_gyro:=true enable_accel:=true --screen