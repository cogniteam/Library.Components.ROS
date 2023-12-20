FROM ros:noetic

COPY ./object_detection /object_detection_ws/src/object_detection

COPY ./object_msgs /object_detection_ws/src/object_msgs


WORKDIR /object_detection_ws


RUN apt update && \
    apt-get install pip -y

RUN apt-get install ros-noetic-cv-bridge -y  &&  \ 
    rm /var/lib/apt/lists/* -rf


RUN . /opt/ros/noetic/setup.sh && catkin_make


RUN python3 -m pip install numpy>=1.20.0  # To ensure compatibility with OpenCV on Raspberry Pi.
RUN python3 -m pip install opencv-python~=4.5.3.56

#RUN echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
#RUN sudo apt-get update && \
    #sudo apt-get install python3-tflite-runtime
RUN pip3 install --extra-index-url https://google-coral.github.io/py-repo/ tflite_runtime

    
#RUN python3 -m pip install --index-url https://google-coral.github.io/py-repo/
#RUN sh setup.sh

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN chmod +x /object_detection_ws/src/object_detection/src/object_detection_node.py

ENTRYPOINT [ "/entrypoint.sh" ]

CMD ["roslaunch" ,"object_detection", "object_detection.launch"]
