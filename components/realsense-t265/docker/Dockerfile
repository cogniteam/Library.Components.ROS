FROM ros:melodic-ros-core
RUN apt -q -qq update && DEBIAN_FRONTEND=noninteractive apt -y install ros-melodic-realsense2-camera ros-melodic-image-transport-plugins && rm -rf /var/lib/apt/lists/*

COPY cogniteam_rs_t265.launch /opt/ros/melodic/share/realsense2_camera/launch/
WORKDIR /home
RUN . /opt/ros/melodic/setup.sh

CMD ["roslaunch", "realsense2_camera", "cogniteam_rs_t265.launch","enable_fisheye1:=true", "enable_fisheye2:=true"]
