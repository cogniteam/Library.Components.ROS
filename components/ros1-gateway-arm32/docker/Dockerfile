FROM arm32v7/ros:noetic

COPY ./empty /gateway_ws/src/empty

WORKDIR /gateway_ws

RUN mkdir devel && touch devel/setup.sh

# RUN . /opt/ros/noetic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

CMD ["roslaunch" ,"empty", "empty.launch"]