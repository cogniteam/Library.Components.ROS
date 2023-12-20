# Bboxes-To-3D-Markers

<img src="./bboxes-to-3d-markers/Cogniteam_CMYK_Social_white_on_aubergine.jpg" alt="bboxes-to-3d-markers" width="400"/>

* Dockerhub image https://hub.docker.com/r/cognimbus/bounding_boxes_objects_to_marker_array
* Supported architectures <b>amd64/arm64</b>
* ROS version <b>melodic
</b>

# Short description
* given depth img and b-boxes, create 3d markers array
License: BSD
git: https://github.com/cognimbus/nimbus.library/tree/master/Library/Components/bboxes-2-threed-markers

# Example usage
```
docker run -it --network=host cognimbus/bounding_boxes_objects_to_marker_array roslaunch objects_pose_estimation objects_poses_estimations.launch map_frame_id:=map static_objects_life_time_seconds:=10
```

# Subscribers
ROS topic | type
--- | ---
/camera/depth/image_rect | sensor_msgs/Image
/camera/depth/image_rect/camera_info | sensor_msgs/CameraInfo
/vision_msg_detected_objects | vision_msgs/Detection2DArray
/openvino_toolkit/detected_objects | object_msgs/ObjectsInBoxes


# Publishers
ROS topic | type
--- | ---
/objects_on_map_markers | visualization_msgs/MarkerArray


# Required tf
base_link--->camera_depth_optical_frame
map--->base_link


# Provided tf
This node does not provide tf


