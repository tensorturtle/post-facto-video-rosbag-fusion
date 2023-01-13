# Launch docker with ROS environment
# and record rosbag
sudo docker run \
	-it \
	--rm \
	--network host \
	-v ${PWD}:/workspace \
	galactic-ros-mcap:latest \
	/workspace/record_topics.sh
