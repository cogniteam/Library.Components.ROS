# Audio_Capture

<img src="./audio_capture/microphone.png" alt="audio_capture" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/ros-audio
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>melodic
</b>

# Short description
* capture audio
License: BSD

# Example usage
```
docker run -it --network=host --privileged cognimbus/ros-audio roslaunch audio_capture capture.launch device:= bitrate:=128 channels:=1 sample_rate:=16000 format:=mp3 sample_format:=S16LE
```

# Subscribers
This node has no subscribers


# Publishers
ROS topic | type
--- | ---
/audio/audio | AudioData
/audio/audio_info | AudioInfo


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


# Audio_Play

<img src="./audio_play/speaker.png" alt="audio_play" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/ros-audio
* Supported architectures <b>arm64/amd64</b>
* ROS version <b>melodic
</b>

# Short description
* play audio
License: BSD

# Example usage
```
docker run -it --network=host --privileged cognimbus/ros-audio roslaunch audio_play play.launch device:= bitrate:=128 channels:=1 sample_rate:=16000 format:=mp3 sample_format:=S16LE
```

# Subscribers
ROS topic | type
--- | ---
/audio/audio | AudioData


# Publishers
This node has no publishers


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


