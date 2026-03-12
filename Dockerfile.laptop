FROM ros:humble-ros-base

# Install CycloneDDS, Git, and build tools
RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    git \
    libboost-python-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Copy the CycloneDDS configuration directly into the image
COPY cyclonedds_laptop.xml /root/cyclonedds.xml

# Set CycloneDDS environment variables
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///root/cyclonedds.xml

# Source ROS 2 setup automatically for interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

RUN pip install casadi scipy

WORKDIR /root/ros2_ws_laptop

# At runtime, ensure the src directory exists, clone the package if missing, then start bash
CMD ["bash", "-c", "mkdir -p src && if [ ! -d 'src/rf2o_laser_odometry' ]; then git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git src/rf2o_laser_odometry; fi && exec bash"]
