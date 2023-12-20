FROM ros:noetic

COPY ./aruco_detection /aruco_detection_ws/src/aruco_detection

WORKDIR /aruco_detection_ws


RUN apt update && \
    apt-get install pip -y
    
RUN pip3 install imutils


RUN rosdep update
RUN DEBIAN_FRONTEND=noninteractive rosdep install --from-paths src --ignore-src --rosdistro=noetic -r -y
RUN . /opt/ros/noetic/setup.sh && catkin_make


RUN python3 -m pip install numpy>=1.20.0  # To ensure compatibility with OpenCV on Raspberry Pi.
#RUN python3 -m pip install opencv-python~=4.5.3.56
RUN pip install opencv-contrib-python

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD ["roslaunch" ,"aruco_detection", "aruco_detection.launch"]