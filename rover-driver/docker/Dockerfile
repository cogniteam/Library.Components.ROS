FROM ros:melodic

SHELL ["/bin/bash", "-c"]

# RUN LINE BELOW TO REMOVE debconf ERRORS (MUST RUN BEFORE ANY apt-get CALLS)
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN apt-get update && apt-get install -y 

RUN apt-get update && apt install python-pip -y
RUN pip install ds4drv
RUN python -m pip install -U six
RUN pip install pyserial

RUN apt install ros-melodic-joy ros-melodic-serial ros-melodic-twist-mux ros-melodic-tf2-geometry-msgs ros-melodic-robot-localization ros-melodic-gmapping ros-melodic-move-base -y

 
COPY ./src /rover_driver_ws/src

WORKDIR /rover_driver_ws/src 
#RUN git clone https://github.com/RoverRobotics/roverrobotics_ros1
#RUN git clone https://github.com/roverrobotics/rr_openrover_stack 
RUN git clone --branch melodic-devel https://github.com/naoki-mizuno/ds4_driver.git

WORKDIR /root/library
RUN git clone https://github.com/RoverRobotics/librover
WORKDIR /root/library/librover/
RUN cmake .
RUN make
RUN sudo make install


WORKDIR /rover_driver_ws

#COPY ./drivers /rover_driver_ws/drivers

#RUN rosdep install --from-paths src --ignore-src -r -y



#RUN python3 setup.py install --prefix ~/.local
#RUN sudo cp udev/50-ds4drv.rules /etc/udev/rules.d/

RUN . /opt/ros/melodic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

#CMD roslaunch roverrobotics_driver zero2.launch
CMD roslaunch rr_rover_zero_driver teleop.launch


