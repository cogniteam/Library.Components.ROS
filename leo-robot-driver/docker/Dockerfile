FROM ros:noetic

COPY ./leo_robot /leo_robot_ws/src

WORKDIR /leo_robot_ws

RUN apt update && rosdep update && rosdep install --from-path src --ignore-src -y && rm /var/lib/apt/lists/* -rf

RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]


 

