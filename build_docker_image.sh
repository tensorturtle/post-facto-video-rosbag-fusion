# Build docker image called 'galactic-ros-mcap' from Dockerfile.galactic

sudo docker build \
    -t galactic-ros-mcap:latest \
    -f Dockerfile.galactic \
    .