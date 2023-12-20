#!/bin/bash
sudo docker build . -t probot_gps
sudo docker run -it --network=host probot_gps roslaunch novatel_oem7_driver oem7_net.launch oem7_ip_addr:=192.168.74.10 oem7_port:=6002 oem7_if:=Oem7ReceiverUdp

