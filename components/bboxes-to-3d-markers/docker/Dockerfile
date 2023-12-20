FROM ros:melodic

COPY ./boxes_objects_to_marker_array /boxes_objects_to_marker_array_ws/src/boxes_objects_to_marker_array
WORKDIR /boxes_objects_to_marker_array_ws

RUN apt update && apt-get install ros-melodic-vision-msgs && rosdep update && rosdep install --from-path src --ignore-src -y && rm /var/lib/apt/lists/* -rf
RUN . /opt/ros/melodic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch objects_pose_estimation objects_poses_estimations.launch
