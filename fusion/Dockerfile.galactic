FROM dustynv/ros:galactic-ros-base-l4t-r35.1.0

# mcap plugin to rosbag2; will be default in next version (iron)
RUN apt-get update && \
	apt-get install -y --no-install-recommends ros-galactic-rosbag2-storage-mcap
RUN echo 'source /opt/ros/galactic/setup.bash' >> /root/.bashrc

WORKDIR /workspace

