FROM ros:melodic

RUN apt-get update && apt-get install -y ros-melodic-slam-toolbox ros-melodic-slam-toolbox-msgs ros-melodic-slam-toolbox-dbgsym && rm -rf /var/lib/apt/lists/*
CMD ["roslaunch", "slam_toolbox", "online_sync.launch"]

