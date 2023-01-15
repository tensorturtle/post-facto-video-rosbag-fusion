# Post-facto Video-Rosbag Fusion

**Problem**: ROS2 saves camera frames individually as still images when recording to a rosbag, which is very inefficient in both storage and processing.

**Solution**: Record camera frames as video file using NVIDIA Jetson's built-in hardware acceleration outside ROS. Record other information as usual using ROS. After the recording is finished, create a new rosbag by merging the videos and initial rosbag.

## Tutorial

### Prerequisites

+ NVIDIA Jetson Nano / Xavier NX (tested) / Xavier AGX with internet connection.
+ V4l2-compatible USB camera or [Raspberry Pi V2 Camera](https://www.raspberrypi.com/products/camera-module-v2/)
+ Strongly recommended: [Run the Jetson from an NVME drive](https://jetsonhacks.com/2020/05/29/jetson-xavier-nx-run-from-ssd/)

Install Docker from [official instructions](https://docs.docker.com/engine/install/ubuntu/)

### Record ROS topics and video outside ROS 

Confirm camera connection. If one camera is connected, it should show up as: `/dev/video0`.

Start one or more ROS2 nodes that publish some data. 

In different terminal window start recording the video outside ROS:

```
cd camera
./csi_recorder_autosplit.sh # or ./usb_recorder_autosplit.sh if using USB camera
```

In a third terminal window, start rosbag recording from within a Docker container:

```
./run_rosbag.sh
```

Both the camera and rosbag will save to `output/` directory.

After a minute or so, press CTRL+C for both the camera and rosbag recording.

### Post-facto Fusion

'''
./run_docker_for_fuser.sh
'''

In the created shell with ROS2 activated, run:

```
python3 fuser.py
```

This will create a new single rosbag file in `output/` directory which contains both the video and the ROS topics.

There are several options for `fuser.py` to help reduce storage, for example. Run `python3 fuser.py --help` to see them.

## Todo
- [ ] Desktop Dockerfile for ROS2 for faster fusing. (Currently only works on Jetson)
- [ ] May 2023: ROS2 Iron release with MCAP as default. Removes need for customized Dockerfile.
