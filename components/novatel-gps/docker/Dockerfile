FROM ros:melodic

#
# Set default shell to bash (following setup.sh is a bash script)
#
SHELL [ "/bin/bash", "-c" ]

RUN apt update && apt install ros-melodic-novatel-oem7-driver -y && rm /var/lib/apt/lists/* -rf

COPY /std_driver_config.xml /opt/ros/melodic/share/novatel_oem7_driver/config/

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

 
#
# Ready !
#
