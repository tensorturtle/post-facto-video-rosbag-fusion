echo "From the Docker container created below, run:"
echo "python3 fuser.py"
echo
sudo docker run \
	-it \
	--rm \
	--network host \
	-v ${PWD}:/workspace \
	-v ${PWD}/output:/output \
	galactic-ros-mcap:latest \