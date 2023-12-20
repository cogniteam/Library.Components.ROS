FROM ros:noetic

COPY ./waypoints_navigation_ws /waypoints_navigation_ws



	
	
	
	
WORKDIR /waypoints_navigation_ws

RUN apt-get update && \ 
    apt install nano -y && \ 
    . /opt/ros/noetic/setup.sh && \     
    rosdep update && rosdep install --from-path src --ignore-src -yr && \
    rm /var/lib/apt/lists/* -rf

RUN . /opt/ros/noetic/setup.sh && catkin_make


	
	
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

 

