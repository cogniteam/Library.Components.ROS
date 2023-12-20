FROM ros:noetic

# copy example files
COPY ./anomaly_ws /anomaly_ws
WORKDIR ./anomaly_ws
RUN . /opt/ros/noetic/setup.sh && catkin_make

# setup entrypoint
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]


CMD ["roslaunch", "anomaly_detection_launch", "train.launch"]
