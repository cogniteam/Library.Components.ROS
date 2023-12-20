FROM ros:melodic

COPY ./person_skeleton_follower /person_skeleton_follower_ws/src/person_skeleton_follower
WORKDIR /person_skeleton_follower_ws

#git clone https://github.com/nlohmann/json.git
#// mkdir build
#//cd build
#// cmake ..
#//make
#//make install
#//ldconfig


RUN apt update && \
    apt-get install -y git && \
    git clone https://github.com/nlohmann/json.git 

WORKDIR /person_skeleton_follower_ws/json
RUN mkdir -p build
WORKDIR /person_skeleton_follower_ws/json/build
RUN cmake ..
RUN make
RUN make install
RUN ldconfig


WORKDIR /person_skeleton_follower_ws/
RUN apt-get update && apt-get install ros-melodic-image-transport-plugins -y  && \
    apt-get install ros-melodic-image-transport -y && \
    apt-get install ros-melodic-cv-bridge -y  && \
    # rosdep update && rosdep install --from-path src --ignore-src -y 
    rm /var/lib/apt/lists/* -rf

RUN . /opt/ros/melodic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch person_skeleton_follower person_skeleton_follower.launch
