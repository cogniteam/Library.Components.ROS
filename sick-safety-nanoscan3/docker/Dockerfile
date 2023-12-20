FROM ros:noetic

RUN . /opt/ros/noetic/setup.sh

RUN apt update && apt-get install -y ros-noetic-sick-safetyscanners

COPY /sick_safetyscanners.launch /opt/ros/noetic/share/sick_safetyscanners/launch/

CMD roslaunch sick_safetyscanners sick_safetyscanners.launch --screen
