# From within a ROS container, start rosbag recording in mcap format
source /opt/ros/galactic/setup.bash
mkdir -p /output/rosbag
cd /output/rosbag
# --storage: use mcap storage
# -d: split files every n seconds 
ros2 bag record \
	--storage mcap \
	-d 60 \
	/image/cam0
