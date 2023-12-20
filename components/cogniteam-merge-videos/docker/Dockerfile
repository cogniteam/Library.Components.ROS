FROM ros:noetic

COPY ./cogniteam-merge-videos /merge_videos_ws/src/cogniteam-merge-videos

WORKDIR /merge_videos_ws

RUN apt update && rosdep update && apt install ros-noetic-image-geometry -y && apt install ros-noetic-image-transport-plugins -y && apt install ros-noetic-tf2-geometry-msgs -y && apt install ros-noetic-geometry -y && rosdep install --from-path src --ignore-src -y
RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

