# Vosk-Speech-To-Text

<img src="./vosk-speech-to-text/speech-text.png" alt="vosk-speech-to-text" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/vosk-speech-to-text
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>noetic
</b>

# Short description
* A ROS package for speech-to-text services based on Vosk

# Example usage
```
docker run -it --network=host --privileged cognimbus/vosk-speech-to-text:latest roslaunch ros_vosk ros_vosk.launch
```

# Subscribers
This node has no subscribers


# Publishers
ROS topic | type
--- | ---
/speech_recognition/final_result | std_msgs/String


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


