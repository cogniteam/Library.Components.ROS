FROM ros:melodic

COPY ./person_follower_rgb /person_follower_rgb_ws/src/person_follower_rgb

WORKDIR /person_follower_rgb_ws


RUN apt update && \
    apt-get install -y git && \
    git clone https://github.com/nlohmann/json.git 

WORKDIR /person_follower_rgb_ws/json
RUN mkdir -p build
WORKDIR /person_follower_rgb_ws/json/build
RUN cmake ..
RUN make
RUN make install
RUN ldconfig



WORKDIR /person_follower_rgb_ws

RUN apt update && apt-get install ros-melodic-image-transport-plugins -y && apt-get install ros-melodic-image-transport -y && apt-get install ros-melodic-cv-bridge -y &&apt-get install ros-melodic-vision-msgs && rosdep update && rosdep install --from-path src --ignore-src -y && rm /var/lib/apt/lists/* -rf
RUN . /opt/ros/melodic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD roslaunch person_follower_rgb person_follower_rgb.launch
