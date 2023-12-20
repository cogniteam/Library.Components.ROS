FROM ros:noetic


WORKDIR /leo_robot_ws/src

RUN apt update && \
    apt upgrade -y && \
    apt install git -y && \
    apt install -y curl
    
RUN curl -sSL https://files.fictionlab.pl/repo/fictionlab.gpg -o /usr/share/keyrings/fictionlab-archive-keyring.gpg 
RUN echo "deb [signed-by=/usr/share/keyrings/fictionlab-archive-keyring.gpg] http://files.fictionlab.pl/repo focal main" | sudo tee /etc/apt/sources.list.d/fictionlab.list > /dev/null
RUN apt update
 

RUN . /opt/ros/noetic/setup.sh && \
    apt install -y ros-noetic-leo && \
    apt install -y ros-noetic-leo-bringup && \
    apt install -y ros-noetic-leo-description && \
    apt install -y ros-noetic-leo-msgs && \
    apt install -y ros-noetic-leo-robot && \
    apt install -y ros-noetic-leo-teleop && \
    apt install -y ros-noetic-leo-fw && \
    apt install -y ros-noetic-leo-fw-dbgsym 
RUN apt install -y ros-noetic-raspicam-node

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]


