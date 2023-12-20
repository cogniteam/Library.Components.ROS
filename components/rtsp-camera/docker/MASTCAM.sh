#!/bin/bash
sudo docker build . -t probot_rtsp
sudo docker run -it --network=host probot_rtsp roslaunch video_stream_opencv camera.launch video_stream_provider:=rtsp://admin@192.168.217.20:554/av0_0 camera_name:=camera_mast
