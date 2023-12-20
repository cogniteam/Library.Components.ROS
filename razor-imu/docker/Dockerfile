FROM ros:melodic

COPY ./razor_imu_m0_driver razor_imu_m0_driver_ws/src/razor_imu_m0_driver


COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

RUN apt-get update && \
    apt-get install ros-melodic-serial -y 
   
    
WORKDIR /razor_imu_m0_driver_ws/

RUN . /opt/ros/melodic/setup.sh && catkin_make

CMD  roslaunch razor_imu_m0_driver driver_node.launch --screen
