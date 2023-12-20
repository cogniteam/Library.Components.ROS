# Audio_Say

<img src="./audio_say/text-to-speech.jpg" alt="audio_say" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/audio_say
* Supported architectures <b>amd64/arm64</b>
* ROS version <b>noetic
</b>

# Short description
* Text to speech from audio_common, gets string, output sound
License: BSD

# Example usage
```
docker run -it --network=host --privileged cognimbus/audio_say roslaunch ros_audio_launch say_string.launch voice:=voice_kal_diphone volume:=1.0 device:= bitrate:=128 channels:=1 sample_rate:=16000 format:=mp3 sample_format:=S16LE --screen
```

# Subscribers
ROS topic | type
--- | ---
/msg_to_play | std_msgs/String


# Publishers
This node has no publishers


# Required tf
This node does not require tf


# Provided tf
This node does not provide tf


