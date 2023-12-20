#FROM ros:melodic

#sudo docker build -t cognimbus/zed .

#docker run --gpus all -it --privileged cognimbus/zed-camera-jetson-xavier


FROM stereolabs/zed:3.4-ros-devel-l4t-r32.3.1



COPY ./zed-ros-wrapper /zed_ws/src/zed-ros-wrapper
WORKDIR /zed_ws


RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && apt-get update  && apt install && rosdep update &&  rosdep install  --rosdistro=melodic --from-path src --ignore-src -yr && rm /var/lib/apt/lists/* -rf
RUN . /opt/ros/melodic/setup.sh &&  catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs -DCUDA_CUDART_LIBRARY=/usr/local/cuda/lib64/stubs -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]


CMD roslaunch zed_wrapper zed.launch




