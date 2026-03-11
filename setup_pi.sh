#!/bin/bash

# Exit immediately if a command fails
set -e

echo "Building the pi-ros-env Docker image..."
docker build -t pi-ros-env -f Dockerfile.pi .

echo "Starting the Raspberry Pi container..."
# Removed --rm to make it persistent
# Added --privileged for GPIO/hardware access
# Added --name to easily reference the container later
docker run -it --net=host --ipc=host --privileged \
    -v "$(pwd)/ros2_ws_pi:/root/ros2_ws_pi" \
    pi-ros-env
