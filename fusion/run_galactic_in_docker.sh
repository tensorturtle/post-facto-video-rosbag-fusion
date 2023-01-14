# Launch docker with ROS environment
# and record rosbag
sudo docker run \
	-it \
	--rm \
	--network host \
	-v ${PWD}:/workspace \
	-v /home/boreal/Repos/post-facto-video-rosbag-fusion/output:/output \
	galactic-ros-mcap:latest