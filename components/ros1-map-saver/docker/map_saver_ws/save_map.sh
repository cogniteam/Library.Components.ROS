#!/bin/bash

cd $MAP_SAVE_PATH

source /opt/ros/noetic/setup.bash
echo $MAP_SAVE_COMMAND
$MAP_SAVE_COMMAND
exec "$@"
