FROM ros:noetic

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y apt-utils
RUN apt-get update && apt-get install -y python3-wstool python3-rosdep ninja-build stow

COPY ./cartographer-slam/src /cartographer_ws/src

WORKDIR /cartographer_ws

RUN rosdep update

RUN DEBIAN_FRONTEND=noninteractive rosdep install --from-paths src --ignore-src --rosdistro=noetic -r -y

RUN src/cartographer/scripts/install_abseil.sh

RUN . /opt/ros/noetic/setup.bash && catkin_make_isolated --install --use-ninja

RUN cp install_isolated/share/cartographer_ros/configuration_files/backpack_2d.lua install_isolated/share/cartographer_ros/configuration_files/my_robot.lua
RUN cp install_isolated/share/cartographer_ros/launch/backpack_2d.launch install_isolated/share/cartographer_ros/launch/my_robot.launch


COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD ["roslaunch", "cartographer_ros", "my_robot.launch"]