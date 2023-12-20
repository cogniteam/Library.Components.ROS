#FROM ros:ROS1
FROM ubuntu

COPY ./burn_script.sh /home/remote_burn_ws/burn_script.sh
WORKDIR /home/remote_burn_ws

RUN apt-get update
RUN apt-get install autoconf pkg-config libusb-1.0 git -y
RUN apt-get install stlink-tools -y
RUN apt-get -y install gcc-arm-none-eabi

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

# CMD ["./burn_script.sh", "https://github.com/kfirs97/elf.git", "STM32_HEllo_World.elf"]