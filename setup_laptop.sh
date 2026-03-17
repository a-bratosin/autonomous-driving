#!/bin/bash

# Exit immediately if a command fails
set -e

xhost +local:root
echo "Building the laptop-ros-env Docker image..."
docker build --network=host -t laptop-ros-env -f Dockerfile . 

echo "Starting the laptop container..."
docker run -it --net=host --ipc=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$(pwd)/ros2_ws_laptop:/root/ros2_ws_laptop" \
    laptop-ros-env
