#FROM ros:melodic

#sudo docker build -t cognimbus/jetson-inference-ros-111 .                              

#sudo docker run --runtime nvidia -it --rm --network host --privileged cognimbus/jetson-inference-ros-111 bash



From dustynv/jetson-inference:r32.5.0

COPY ./ros_deep_learning /ros_deep_learning_ws/src/ros_deep_learning
COPY ./data /jetson-inference/data

# Installing ROS melodic
RUN apt-get update && apt-get install -y lsb-release && apt-get clean all && apt-get install -y gnupg2 && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && apt-get install -y ros-melodic-ros-base && \
    apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential


WORKDIR /ros_deep_learning_ws


RUN apt-get update &&  apt-get install ros-melodic-image-transport -y && \ 
    apt-get install ros-melodic-vision-msgs -y && \ 
    apt-get install ros-melodic-image-transport-plugins -y &&  \  
    rm /var/lib/apt/lists/* -rf && \  
    pip install catkin_pkg defusedxml rospkg netifaces

RUN . /opt/ros/melodic/setup.sh && \ 
    catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs -DCUDA_CUDART_LIBRARY=/usr/local/cuda/lib64/stubs -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

#RUN <jetson-inference>/tools
#           $ ./download-models.sh


#CMD roslaunch ros_deep_learning detectnet.ros1.launch prototxt_path:=/ros_deep_learning_ws/src/data/networks/detectnet.prototxt model_path:=/ros_deep_learning_ws/src/data/networks/SSD-Mobilenet-v2/ssd_mobilenet_v2_coco.uff --screen
