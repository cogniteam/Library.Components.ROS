#!/bin/bash
sudo docker build . -t probot_rtsp_front_cam
sudo docker run -it --network=host probot_rtsp_front_cam roslaunch video_stream_opencv camera.launch video_stream_provider:=rtsp://admin@192.168.217.188:554 camera_name:=camera_thermal
