#!/bin/sh

. /sweep_ws/devel/setup.sh
exec roslaunch sweep_ros sweep2scan.launch

exec "$@"