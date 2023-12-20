#FROM ros:melodic
FROM cognimbus/ros1-autopilot-bridge:navSatFix
WORKDIR /autopilot_bridge_ws
RUN rm -r src/*

WORKDIR /autopilot_bridge_ws/src

RUN git clone https://github.com/mikeclement/autopilot_bridge.git

COPY /launch /autopilot_bridge_ws/src/autopilot_bridge
RUN rm /autopilot_bridge_ws/src/autopilot_bridge/src/mavbridge.py
COPY docker/mavbridge.py /autopilot_bridge_ws/src/autopilot_bridge/src

WORKDIR /autopilot_bridge_ws

RUN . /opt/ros/melodic/setup.sh && catkin_make

COPY entrypoint.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]