FROM ros:noetic


RUN apt update && apt-get install ros-noetic-nmea-* -y

RUN . /opt/ros/noetic/setup.sh 


CMD roslaunch nmea_navsat_driver nmea_serial_driver.launch baud:=9600 --screen