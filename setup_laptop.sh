#!/bin/bash

# Exit immediately if a command fails
set -e

echo "Building the laptop-ros-env Docker image..."
docker build -t laptop-ros-env -f Dockerfile.laptop .

echo "Starting the laptop container..."
docker run -it --net=host --ipc=host \
    -v "$(pwd)/ros2_ws_laptop:/root/ros2_ws_laptop" \
    laptop-ros-env
