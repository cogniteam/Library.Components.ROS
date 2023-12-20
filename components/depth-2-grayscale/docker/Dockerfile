FROM ros:melodic

COPY ./depth_to_grayscale depth_to_grayscale_ws/src/depth_to_grayscale

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

RUN apt-get update && apt-get install ros-melodic-image-transport-plugins -y && apt-get install ros-melodic-image-transport -y && apt-get install ros-melodic-cv-bridge -y && rm /var/lib/apt/lists/* -rf
WORKDIR /depth_to_grayscale_ws/

RUN . /opt/ros/melodic/setup.sh && catkin_make

CMD roslaunch depth_to_grayscale depth_to_grayscale.launch --screen