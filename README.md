# Cogniteam Component library for ROS noetic
This library contains open dockerized components for ROS
If you wish to use ROS2 check out our [ROS2 library](https://github.com/cogniteam/Library.Components.ROS2/tree/master)
# ROSCon 2023 

To participate in the contest and stand a chance to win a [Leo Rover](https://www.leorover.tech/), start by forking our library. Following this, you have two options to proceed:

1. Initiate a merge request to include a folder containing your component's Dockerfile. For guidance on this, please [refer to these instructions](#option-1-add-a-folder-with-your-components-dockerfile).
   
2. Create a merge request to add your git repository details to the `ContributedComponents.MD` file. Detailed steps can be found [here](#option-2-add-your-git-repository-to-contributedcomponentsmd).
# Cogniteam’s Components Table
Image | Link
--- | ---
<img src="./components/aruco-code-detection/aruco-code-detection/aruco_detection.png" alt="aruco-code-detection" width="40"/> | [aruco-code-detection](components/aruco-code-detection)
<img src="./components/audio_say/audio_say/text-to-speech.jpg" alt="audio_say" width="40"/> | [audio_say](components/audio_say)
<img src="./components/autopilot_bridge/autopilot_bridge/Autopilot_bridge.png" alt="autopilot_bridge" width="40"/> | [autopilot_bridge](components/autopilot_bridge)
<img src="./components/caffe-object-detection/caffe-object-detection/caffe-object-detection.jpg" alt="caffe-object-detection" width="40"/> | [caffe-object-detection](components/caffe-object-detection)
<img src="./components/cartographer-slam/cartographer-slam/cartographer-slam.png" alt="cartographer-slam" width="40"/> | [cartographer-slam](components/cartographer-slam)
<img src="./components/cogniteam-anomaly-detection/cogniteam-anomaly-detection/falling-off-chart.jpg" alt="cogniteam-anomaly-detection" width="40"/> | [cogniteam-anomaly-detection](components/cogniteam-anomaly-detection)
<img src="./components/cogniteam-merge-videos/cogniteam-merge-videos/Cogniteam_CMYK_Social_white_on_aubergine.jpg" alt="cogniteam-merge-videos" width="40"/> | [cogniteam-merge-videos](components/cogniteam-merge-videos)
<img src="./components/cogniteam-random-goals/cogniteam-random-goals/Cogniteam_CMYK_Social_white_on_aubergine.jpg" alt="cogniteam-random-goals" width="40"/> | [cogniteam-random-goals](components/cogniteam-random-goals)
<img src="./components/custom-message-talker-listener/custom-message-listener/nimbusc.jpg" alt="custom-message-listener" width="40"/> | [custom-message-listener](components/custom-message-talker-listener)
<img src="./components/custom-message-talker-listener/custom-message-talker/nimbusc.jpg" alt="custom-message-talker" width="40"/> | [custom-message-talker](components/custom-message-talker-listener)
<img src="./components/custom-ros-service/custom-service-server/nimbusc.jpg" alt="custom-service-server" width="40"/> | [custom-service-server](components/custom-ros-service)
<img src="./components/custom-ros-service/custom-service-client/nimbusc.jpg" alt="custom-service-client" width="40"/> | [custom-service-client](components/custom-ros-service)
<img src="./components/deegoo-fpv-gps/deegoo-fpv-gps/gps.jpeg" alt="deegoo-fpv-gps" width="40"/> | [deegoo-fpv-gps](components/deegoo-fpv-gps)
<img src="./components/faces-landsmarks-mediapipe/faces-landsmarks-mediapipe/faces.jpg" alt="faces-landsmarks-mediapipe" width="40"/> | [faces-landsmarks-mediapipe](components/faces-landsmarks-mediapipe)
<img src="./components/generic-webcam-with-mic/generic-webcam-with-mic/generic-webcam-driver.jpg" alt="generic-webcam-with-mic" width="40"/> | [generic-webcam-with-mic](components/generic-webcam-with-mic)
<img src="./components/hamster-wandering/hamster-wandering/nimbusc.png" alt="hamster-wandering" width="40"/> | [hamster-wandering](components/hamster-wandering)
<img src="./components/hands_pose_detection/hands_pose_detection/hand_landmarks.png" alt="hands_pose_detection" width="40"/> | [hands_pose_detection](components/hands_pose_detection)
<img src="./components/image-topic-to-rtsp/image-topic-to-rtsp/Cogniteam_CMYK_Social_white_on_aubergine.jpg" alt="image-topic-to-rtsp" width="40"/> | [image-topic-to-rtsp](components/image-topic-to-rtsp)
<img src="./components/imu-filter-madgwick/imu-filter-madgwick/cogniteam_imu_tools.jpg" alt="imu-filter-madgwick" width="40"/> | [imu-filter-madgwick](components/imu-filter-madgwick)
<img src="./components/joystick-teleop-twist/joystick-teleop-twist/nimbusc.jpg" alt="joystick-teleop-twist" width="40"/> | [joystick-teleop-twist](components/joystick-teleop-twist)
<img src="./components/leg-detector/leg-detector/nimbusc.png" alt="leg-detector" width="40"/> | [leg-detector](components/leg-detector)
<img src="./components/nextVision_cam/nextVision_cam/nimbusc.jpg" alt="nextVision_cam" width="40"/> | [nextVision_cam](components/nextVision_cam)
<img src="./components/path-publisher/path-publisher/cogniteam.jpg" alt="path-publisher" width="40"/> | [path-publisher](components/path-publisher)
<img src="./components/path-waypoints-navigation/path-waypoints-navigation/cogniteam.jpg" alt="path-waypoints-navigation" width="40"/> | [path-waypoints-navigation](components/path-waypoints-navigation)
<img src="./components/richbeam-lakibeam/richbeam-lakibeam/lakibeam.png" alt="richbeam-lakibeam" width="40"/> | [richbeam-lakibeam](components/richbeam-lakibeam)
<img src="./components/ros-serial-st-python/ros-serial-st-python/stst.jpg" alt="ros-serial-st-python" width="40"/> | [ros-serial-st-python](components/ros-serial-st-python)
<img src="./components/ros-tutorials-listener/ros-tutorials-listener/nimbusc.jpg" alt="ros-tutorials-listener" width="40"/> | [ros-tutorials-listener](components/ros-tutorials-listener)
<img src="./components/ros-tutorials-talker/ros-tutorials-talker/nimbusc.jpg" alt="ros-tutorials-talker" width="40"/> | [ros-tutorials-talker](components/ros-tutorials-talker)
<img src="./components/ros1-gateway-arm32/ros1-gateway-arm32/nimbusc.jpg" alt="ros1-gateway-arm32" width="40"/> | [ros1-gateway-arm32](components/ros1-gateway-arm32)
<img src="./components/ros1-gateway-launch/ros1-gateway-launch/nimbusc.jpg" alt="ros1-gateway-launch" width="40"/> | [ros1-gateway-launch](components/ros1-gateway-launch)
<img src="./components/sick-safety-nanoscan3/sick-safety-nanoscan3/nanoscan3.png" alt="sick-safety-nanoscan3" width="40"/> | [sick-safety-nanoscan3](components/sick-safety-nanoscan3)
<img src="./components/slamtec-rplidar-driver/rplidar-a1/slamtec-rplidar-a1-driver.jpg" alt="rplidar-a1" width="40"/> | [rplidar-a1](components/slamtec-rplidar-driver)
<img src="./components/slamtec-rplidar-driver/rplidar-a3/slamtec-rplidar-a3-driver.jpg" alt="rplidar-a3" width="40"/> | [rplidar-a3](components/slamtec-rplidar-driver)
<img src="./components/slamtec-rplidar-driver/rplidar-a2/slamtec-rplidar-a2-driver.jpg" alt="rplidar-a2" width="40"/> | [rplidar-a2](components/slamtec-rplidar-driver)
<img src="./components/slamtec-rplidar-driver/rplidar-s1/slamtec-rplidar-s1-driver.jpg" alt="rplidar-s1" width="40"/> | [rplidar-s1](components/slamtec-rplidar-driver)
<img src="./components/slamtec-rplidar-driver/rplidar-s2/slamtec-rplidar-s2-driver.jpg" alt="rplidar-s2" width="40"/> | [rplidar-s2](components/slamtec-rplidar-driver)
<img src="./components/spot-bridge/spot-bridge/spot.jpeg" alt="spot-bridge" width="40"/> | [spot-bridge](components/spot-bridge)
<img src="./components/stm-remote-burn/stm-remote-burn/nimbusc.jpg" alt="stm-remote-burn" width="40"/> | [stm-remote-burn](components/stm-remote-burn)
<img src="./components/tflite-object-detection/tflite-object-detection/object_detection.png" alt="tflite-object-detection" width="40"/> | [tflite-object-detection](components/tflite-object-detection)
<img src="./components/turtlebot3/turtlebot3-lidar/turtlebot.jpeg" alt="turtlebot3-lidar" width="40"/> | [turtlebot3-lidar](components/turtlebot3)
<img src="./components/turtlebot3/turtlebot3-picamera/turtlebot.jpeg" alt="turtlebot3-picamera" width="40"/> | [turtlebot3-picamera](components/turtlebot3)
<img src="./components/turtlebot3/turtlebot3-driver/turtlebot.jpeg" alt="turtlebot3-driver" width="40"/> | [turtlebot3-driver](components/turtlebot3)
<img src="./components/turtlebot3/turtlebot3-slam/turtlebot.jpeg" alt="turtlebot3-slam" width="40"/> | [turtlebot3-slam](components/turtlebot3)
<img src="./components/turtlebot3/turtlebot3-navigation/turtlebot.jpeg" alt="turtlebot3-navigation" width="40"/> | [turtlebot3-navigation](components/turtlebot3)
<img src="./components/vosk-speech-to-text/vosk-speech-to-text/speech-text.png" alt="vosk-speech-to-text" width="40"/> | [vosk-speech-to-text](components/vosk-speech-to-text)

# Contributed Components Table 
Image | Link
--- | ---
<img src="https://github.com/ptrks/ROS-YDLidar-x4-docker/blob/master/doc/example.gif" alt="YDLidar-x4" width="40"/> | [ROS-YDLidar-x4](https://github.com/ptrks/ROS-YDLidar-x4-docker)
# Contribution
 If you wish to contribute by adding a new component to our library as part of our ongoing competition, please follow the instructions below:

 ## Prerequisites

Before you begin, ensure you have met the following requirements:

- You have a [GitHub](https://github.com) account.
- You have installed [Git](https://git-scm.com/).
- You have installed [Docker](https://www.docker.com/get-started).

## Forking and Cloning the Repository

1. **Fork the Repository**: Click on the 'Fork' button on the upper right-hand side of the page. A copy of the repository will be created on your personal GitHub account.
2. **Clone the Repository**: Clone the forked repository to your local machine by running:
   ```bash
   git clone https://github.com/cognimbus/Nimbus.Library.Components.ROS.git
   ```

## Adding a New Component

### Option 1: Add a folder with your component's Dockerfile

#### 1. **Prepare Your Component Structure**
   - `comp_name`: Directory for your component
      - `docker`: Contains code and the Docker file
      - `img_file`: image represents the component

#### 2. **Create and Test Your Dockerfile**
   - Navigate to the cloned repository on your local machine.
   - Create a new Dockerfile with the required configurations for the ROS/ROS2 application you wish to containerize.
   - Test your Dockerfile locally with:
     ```bash
     docker build -t ros_app:<tag> .
     docker run --rm -it ros_app:<tag>
     ```

#### 3. **Place Your Files in the Directory Structure**
   - Place the Dockerfile and code into the appropriate directory structure within `comp_name/docker`.

#### 4. **Commit Your Changes**
   - After testing, commit your changes:
     ```bash
     git add .
     git commit -m "Your detailed commit message"
     ```
### Option 2: Add your git repository to ContributedComponents.MD

If you already have a git repository with a Dockerfile, simply add a link in ContributedComponents.MD and request to merge it. Make sure to add a relevant image from your git and make sure your git includes a valid Dockerfile that uses this version of ROS. 

#### Step 1: Prepare Your Image and Repository URL

Before adding a new row to the table, make sure you have the following:

1. **Image URL**: The URL of the image that represents your component. This should be hosted inside your Git repository. You can obtain the URL by navigating to the image file in your Git repository (e.g., on GitHub) and copying the URL.
   
2. **Repository URL**: The URL of your Git repository where the component is hosted.

#### Step 2: Add a New Row to the Table

To add a new component to the table, follow these steps:

1. **Open the Markdown File**: Open the markdown file where the table is located.

2. **Add a New Row**: Add a new row to the table with the following format:

   ```markdown
   Image | Link
   --- | ---
   <img src="IMAGE_URL" alt="COMPONENT_NAME" width="40"/> | [COMPONENT_NAME](REPOSITORY_URL)
   ```

3. **Replace Placeholders**: Replace `IMAGE_URL`, `COMPONENT_NAME`, and `REPOSITORY_URL` with the actual values:
   
   - `IMAGE_URL`: The URL of the image you prepared in step 1.
   - `COMPONENT_NAME`: The name of your component.
   - `REPOSITORY_URL`: The URL of your Git repository.

## Submitting a Merge Request

1. **Push Your Changes**: Push to your forked repository:
   ```bash
   git push origin <ros-distro>
   ```
2. **Create a Pull Request**: Navigate to your forked repository's GitHub page, click 'Pull request', and write a detailed comment.
3. **Submit**: Click 'Submit pull request'.

## Support and Contact

If you have questions or encounter issues, open an issue in the repository, and one of our maintainers will get back to you as soon as possible. Thank you for your contribution!

---

- [Nimbus Library Components for ROS](https://github.com/cognimbus/Nimbus.Library.Components.ROS)
- [Nimbus Library Components for ROS2](https://github.com/cognimbus/Nimbus.Library.Components.ROS2)