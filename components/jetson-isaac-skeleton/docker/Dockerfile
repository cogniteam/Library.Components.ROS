#FROM ros:melodic
# FROM nvcr.io/nvidia/l4t-base:r32.3.1
#FROM cognimbus/ros_deep_learning_jetson_inference
#FROM koenlek:ros-nvidia

FROM dustynv/jetson-inference:r32.5.0


RUN apt-get update && \
    apt-get install -y curl && \
    apt-get install -y gnupg2 && \
    apt install -y lsb-core && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && \
    apt install -y ros-melodic-ros-base && \
    apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential && \
    apt-get install python-defusedxml && \ 
    rm /var/lib/apt/lists/* -rf

RUN echo "export PATH=/usr/local/cuda/bin:$PATH" >> ~/.bashrc && \
    echo "export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH" >> ~/.bashrc && \
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
#    . ~/.bashrc

RUN . /opt/ros/melodic/setup.sh

COPY openpose_inference-pkg/ /openpose-pkg-run

WORKDIR /openpose-pkg-run

RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/openpose-pkg-run/packages/ml/"

RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/openpose-pkg-run/packages/ros_bridge/"

CMD ["./run", "packages/skeleton_pose_estimation/apps/openpose/run_openpose_inference"]

#RUN apt-get install ros-melodic-image-transport -y && \
#    apt-get install ros-melodic-vision-msgs -y && \
#    apt-get install ros-melodic-image-transport-plugins -y &&  \
#    rm /var/lib/apt/lists/* -rf 
