#!/bin/sh

export LYNX_PATH=/home/lynx/lynx_ws/src/lynx
export LYNX_IP=10.0.2.25

export ROS_MASTER_URI=http://${LYNX_IP}:11311
export ROS_IP=${LYNX_IP}

export PS1="\[\033[38;5;75m\]$(tput bold)\u@\[\033[38;5;215m\]\h\[\033[38;5;15m\]:\[\033[38;5;7m\]\w\[\033[38;5;6m\]:\[\033[38;5;15m\] \[$(tput sgr0)\]"

source /opt/intel/openvino/bin/setupvars.sh

export CPU_EXTENSION_LIB=/opt/intel/openvino/deployment_tools/inference_engine/samples/build/intel64/Release/lib/libcpu_extension.so
export GFLAGS_LIB=/opt/intel/openvino/deployment_tools/inference_engine/samples/build/intel64/Release/lib/libgflags_nothreads.a

source /home/lynx/lynx_ws/devel/setup.bash

export CC="ccache clang"
export CXX="ccache clang++"
