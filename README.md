# Post-facto Video-Rosbag Fusion

## Work in Progress

- [X] NVENC hardware encoding and splitting
- [X] Record rosbag as MCAP file (parallel_recording/rosbag_galactic)
- [ ] Parse video into MCAP message and write to it: https://mcap.dev/guides/python/ros2.html
- [ ] (bonus) split rosbag file by time for data robustness

*ROS2 and ROS are used interchangeably. Both refer to ROS2.*

**Problem**: ROS2 saves camera frames individually as still images when recording to a rosbag, which is very inefficient in both storage and processing.

**Solution**: Record camera frames as video file using NVIDIA Jetson's built-in hardware acceleration outside ROS. Record other information as usual using ROS. After the recording is finished, create a new rosbag by merging the videos and initial rosbag.

[diagram]

## Tutorial

### Prerequisites

+ NVIDIA Jetson Nano / Xavier NX (tested) / Xavier AGX with internet connection.
+ [Raspberry Pi V2 Camera](https://www.raspberrypi.com/products/camera-module-v2/)
+ Strongly recommended: [Run the Jetson from an NVME drive](https://jetsonhacks.com/2020/05/29/jetson-xavier-nx-run-from-ssd/)

### Parallel Recording

Connect the Raspberry Pi V2 Camera to the CSI connector on the Jetson Nano/NX Developer Kit. Confirm connection by rebooting and checking that `/dev/video0` exists.

We use [ROS2 Humble] inside a [Docker] container for quick setup.

Install Docker from [official instructions](https://docs.docker.com/engine/install/ubuntu/)

Start a "hello world" ROS node from within a Docker container. It may take several minutes to download the image.:

```
parallel_recording/run_ros_docker.sh
```

In another terminal window start recording the video outside ROS:

```
parallel_recording/run_camera_recorder.sh
```

In a third terminal window, start rosbag recording from within a Docker container:

```
parallel_recording/run_rosbag.sh
```

After a minute or so, press CTRL+C to stop rosbag recording.

### Post-facto Fusion

If the above parallel recording step was successful, you will find a directory inside `output/` that contains a `.mp4` video file and `.db3` rosbag file. Note the absolute file paths.

Run the fusion script with the absolute paths to the video and rosbag files as such:

```
fusion/fuse.sh /video/file/path.mp4 /rosbag/file/path.db3
```

