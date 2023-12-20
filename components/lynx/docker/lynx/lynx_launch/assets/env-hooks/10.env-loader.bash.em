#!/bin/bash


#
# Prepend to PATH env the path to our bin directory
#
_ALIASES_PATH="$(rospack find lynx_launch)/assets/bin/"
[[ ":${PATH}:" =~ ":${_ALIASES_PATH}:" ]] || export PATH="${_ALIASES_PATH}:${PATH}"

# export -f source_workspace

echo "lynx2 ROS environment loaded"
