FROM ros:noetic

# COPY src /home/lakibeam1_ws/src

WORKDIR /home/lakibeam1_ws

ENV DEBIAN_FRONTEND=noninteractive
RUN apt update -y && apt upgrade -y && apt-get -y install git && \
    echo "31\n31\n1\n" | apt install -y ros-noetic-pcl-conversions && echo "31\n21" | apt install -y ros-noetic-pcl-ros

RUN git clone https://gitee.com/richbeam/Lakibeam1-ROS1.git src
# RUN echo "31\n31\n1\n" | apt install -y ros-noetic-pcl-conversions && echo "31\n21" | apt install -y ros-noetic-pcl-ros

RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]