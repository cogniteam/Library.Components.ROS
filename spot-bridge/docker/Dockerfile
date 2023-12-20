FROM ros:noetic

#https://github.com/heuristicus/spot_ros

RUN apt update && apt install wget -y && apt install git-all -y
RUN wget https://raw.githubusercontent.com/clearpathrobotics/public-rosdistro/master/rosdep/50-clearpath.list -O /etc/ros/rosdep/sources.list.d/50-clearpath.list 
RUN apt update && apt install -y python3-pip bridge-utils 
RUN pip3 install cython  
RUN pip3 install bosdyn-client && pip3 install bosdyn-mission 
RUN pip3 install bosdyn-api && pip3 install bosdyn-core 
RUN pip3 install empy && pip3 install transforms3d
#RUN apt-get install libbullet-dev -y


COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

COPY ./spot_ros sopt_ws/src/spot_ros

WORKDIR sopt_ws
ENV DEBIAN_FRONTEND=noninteractive

RUN apt install ros-noetic-interactive-marker-twist-server -y && \
    apt install ros-noetic-twist-mux -y && \
    apt install ros-noetic-teleop-twist-joy -y  && \
    apt install ros-noetic-joy -y && \
    apt install ros-noetic-robot-state-publisher -y && \
    apt install ros-noetic-xacro -y && \
    apt install ros-noetic-rviz -y && \
    apt-get install qt5-default -y &&  apt install qttools5-dev -y && \
    apt-get install ros-noetic-cv-bridge -y && \
    apt-get install python-is-python3

RUN cd src && \
    cd spot_ros && git clone https://github.com/bdaiinstitute/spot_wrapper.git && \
    pip3 install -e spot_wrapper && cd .. && cd ..

RUN . /opt/ros/noetic/setup.sh && catkin_make -j1

# /bluetooth_teleop/cmd_vel
# /bluetooth_teleop/joy
# /bluetooth_teleop/joy/set_feedback
# /cmd_vel
# /diagnostics
# /joint_states
# /rosout
# /rosout_agg
# /spot/cmd_vel
# /tf
# /tf_static
# /twist_marker_server/cmd_vel
# /twist_marker_server/feedback
# /twist_marker_server/update
# /twist_marker_server/update_full

# the network 
#Remove any wired network configuration files from /etc/netplan. 
#Wireless configuration files may be retained. 
#Create the file /etc/netplan/50-ethernet-bridge.yaml with the following contents: 
# Bridge together all physical ethernet ports and allow them to operate simultaneously on:
# - 192.168.131.1/24 for ROS
# - 192.168.50.1/24 for communicating with the Spot base platform
# - dhcp for wired external internet access
#network:
#  version: 2
#  renderer: networkd
#  ethernets:
#    bridge_eth:
#      dhcp4: no
#      dhcp6: no
#      match:
#        name: eth*
#    bridge_eno:
#      dhcp4: no
#      dhcp6: no
#      match:
#        name: eno*
#    bridge_enp:
#      dhcp4: no
#      dhcp6: no
#      match:
#        name: enp*
#    bridge_enx:
#      dhcp4: no
#      dhcp6: no
#      match:
#        name: enx*
#  bridges:
#    br0:
#      dhcp4: yes
#      dhcp6: no
#      interfaces: [bridge_eth, bridge_eno, bridge_enp, bridge_enx]
#      addresses:
#        - 192.168.50.1/24
#        - 192.168.131.1/24   

# COPY entrypoint.sh /entrypoint.sh
# RUN chmod +x /entrypoint.sh

# ENTRYPOINT [ "/entrypoint.sh" ]

# CMD roslaunch leg_tracker joint_leg_tracker.launch