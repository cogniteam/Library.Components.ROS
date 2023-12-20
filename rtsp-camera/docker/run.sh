#!/bin/bash
sudo docker build . -t probot_rtsp
sudo docker run -it --network=host probot_rtsp roslaunch video_stream_opencv camera.launch video_stream_provider:=rtsp://admin@192.168.88.1:554/av0_0