# From within a ROS container, start rosbag recording in mcap format
source /opt/ros/galactic/setup.bash
ros2 bag record --storage mcap \
	/image/cam0
