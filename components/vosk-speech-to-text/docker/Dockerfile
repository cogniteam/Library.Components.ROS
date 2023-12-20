FROM ros:noetic

COPY ./src /ros_vosk_ws/src
WORKDIR ros_vosk_ws
COPY checkArch.sh /checkArch.sh

RUN apt-get update && sudo apt-get install python3
RUN apt-get -y install python3-pip && pip3 install sounddevice 
RUN apt-get update
RUN ../checkArch.sh
RUN apt-get -y install python3-tk
RUN pip install urllib3 && pip3 install requests 
RUN apt install espeak -y && pip3 install pyttsx3 && pip3 install bs4 && pip3 install lxml
RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]


CMD roslaunch ros_vosk ros_vosk.launch 
