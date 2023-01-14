# From within a ROS container, start rosbag recording in mcap format
source /opt/ros/galactic/setup.bash
cd /output
# --storage: use mcap storage
# -d: split files every n seconds 
ros2 bag record \
	--storage mcap \
	/image/cam0
