#!/bin/bash

# Oprește execuția imediat dacă o comandă eșuează
set -e

# Permite accesul containerului la serverul X11 local pentru GUI
xhost +local:root

echo "Building the jetson-ros-env Docker image..."
docker build --network=host -t jetson-ros-env -f Dockerfile.jetson .

echo "Starting the Jetson container..."
docker run -it --rm \
    --runtime nvidia \
    --net=host \
    --ipc=host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$(pwd)/ros2_ws_jetson:/root/ros2_ws_jetson" \
    jetson-ros-env
while sleep 1; do echo "working"; done
