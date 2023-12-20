#!/bin/sh


export DOCKER_CLI_EXPERIMENTAL=enabled


#
# on the terminal (inside the folder of the Dockerfile
# 

sudo docker buildx create --name haystack-builder 
sudo docker buildx use haystack-builder
sudo docker run --privileged --rm tonistiigi/binfmt --install all
sudo docker buildx inspect --bootstrap


#
# on the terminal (inside the folder of the Dockerfile
#


sudo docker buildx build --platform linux/amd64,linux/arm64 -t cognimbus/ubiquity-driver:latest --push .

