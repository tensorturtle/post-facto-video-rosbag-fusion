# Post-facto Video-Rosbag Fusion

*ROS2 and ROS are used interchangeably. Both refer to ROS2 Humble.*

**Problem**: ROS2 saves camera frames individually as still images when recording to a rosbag, which is very inefficient in both storage and processing.

**Solution**: Record camera frames as video file using NVIDIA Jetson's built-in hardware acceleration outside ROS. Record other information as usual using ROS. After the recording is finished, create a new rosbag by merging the videos and initial rosbag.

[diagram]

## Demonstration

