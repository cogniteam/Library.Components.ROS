FROM ros:kinetic-ros-core
RUN apt-get update && apt-get install -y build-essential ros-kinetic-pcl-conversions ros-kinetic-pointcloud-to-laserscan && rm -rf /var/lib/apt/lists/*
COPY /sweep-libs /

WORKDIR sweep-sdk/libsweep
RUN mkdir -p build
WORKDIR /sweep-sdk/libsweep/build
RUN cmake .. -DCMAKE_BUILD_TYPE=Release
RUN cmake --build .
RUN cmake --build . --target install
RUN ldconfig

WORKDIR /
RUN mkdir sweep_ws
WORKDIR /sweep_ws
RUN mkdir src
COPY /sweep_pkg /sweep_ws/src
RUN . /opt/ros/kinetic/setup.sh && \
    catkin_make

WORKDIR /
COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh
CMD /entrypoint.sh 

