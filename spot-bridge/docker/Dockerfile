FROM ros:noetic

#https://github.com/heuristicus/spot_ros

RUN apt update --fix-missing && apt install wget -y && apt install git-all -y
RUN wget https://raw.githubusercontent.com/clearpathrobotics/public-rosdistro/master/rosdep/50-clearpath.list -O /etc/ros/rosdep/sources.list.d/50-clearpath.list 
RUN apt update && apt install -y \
    python3-pip \
    bridge-utils \
    cython

RUN pip3 install bosdyn-client \
    bosdyn-mission \
    bosdyn-api \
    bosdyn-choreography-client \
    bosdyn-core \
    empy \
    transforms3d \
    opencv-python
#RUN apt-get install libbullet-dev -y

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

WORKDIR /spot_ws
ENV DEBIAN_FRONTEND=noninteractive

COPY ssh/ /root/.ssh

# Add GitHub to known_hosts to avoid host verification prompt
RUN ssh-keyscan github.com >> /root/.ssh/known_hosts

RUN mkdir src && cd src && \
    git clone --recurse-submodule https://github.com/heuristicus/spot_ros.git 

RUN rm /root/.ssh/*

RUN apt install -y \
    ros-noetic-interactive-marker-twist-server \
    ros-noetic-twist-mux \
    ros-noetic-teleop-twist-joy \
    ros-noetic-joy \
    ros-noetic-robot-state-publisher \
    ros-noetic-xacro \
    ros-noetic-rviz \
    ros-noetic-cv-bridge \
    qt5-default \
    qttools5-dev \
    ros-noetic-cv-bridge

COPY cogniteam-packages /spot_ws/src/spot_ros

RUN mv src/spot_ros/driver.launch src/spot_ros/spot_driver/launch/ && \
    cd src/spot_ros/spot_wrapper && pip install .

RUN . /opt/ros/noetic/setup.sh && catkin_make -j1