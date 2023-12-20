FROM ros:noetic

COPY ./object_detection /object_detection_ws/src/object_detection
COPY ./object_msgs /object_detection_ws/src/object_msgs

#COPY ./image_publisher /object_detection_ws/src/image_publisher


WORKDIR /object_detection_ws


RUN apt update && \
    apt-get install pip -y
    
RUN pip3 install imutils

RUN apt-get install ros-noetic-cv-bridge -y  &&  \
    apt-get install ros-noetic-image-transport && \
    apt-get install ros-noetic-image-geometry && \
    rm /var/lib/apt/lists/* -rf


RUN . /opt/ros/noetic/setup.sh && catkin_make


RUN python3 -m pip install numpy>=1.20.0  # To ensure compatibility with OpenCV on Raspberry Pi.
RUN python3 -m pip install opencv-python~=4.5.3.56

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN chmod +x /object_detection_ws/src/object_detection/src/object_detection_node.py

ENTRYPOINT [ "/entrypoint.sh" ]

CMD ["roslaunch" ,"object_detection", "object_detection.launch"]
