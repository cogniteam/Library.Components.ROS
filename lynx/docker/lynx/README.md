Lynx platform software
======================

### Installation

1. RealSense SDK
    https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages

1. RealSense ROS drivers
    ```
    sudo apt install ros-kinetic-realsense2-camera
    ```
1. Install systemd lynx service  
    1. Copy ```lynx_launch/assets/systemd/lynx.service``` to ```/etc/systemd/system/``` or create a symlink

    1. Enable service startup and run it:  
       ```
       sudo systemctl enalbe lynx
       sudo systemctl start lynx
       ```


1. TBD


