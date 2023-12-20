FROM ros:noetic

COPY src /home/lakibeam1_ws/src

WORKDIR /home/lakibeam1_ws

RUN apt update -y && apt upgrade -y

ENV DEBIAN_FRONTEND=noninteractive
RUN echo "31\n31\n1\n" | apt install -y ros-noetic-pcl-conversions && echo "31\n21" | apt install -y ros-noetic-pcl-ros

# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"
RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]
