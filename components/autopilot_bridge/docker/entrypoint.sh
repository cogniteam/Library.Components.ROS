#!/bin/bash
set -e
# setup ros environment
source "/opt/ros/melodic/setup.bash"
source "/autopilot_bridge_ws/devel/setup.bash"
exec "$@"