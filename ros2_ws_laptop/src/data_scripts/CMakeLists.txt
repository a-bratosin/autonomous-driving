cmake_minimum_required(VERSION 3.5)
project(data_scripts)

# Required for Python packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(script_msgs REQUIRED)  # <-- your custom message package

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(
  PROGRAMS
    data_scripts/ekf_node.py
    data_scripts/trajectory_plotter.py
    data_scripts/mpc_node.py
  DESTINATION lib/${PROJECT_NAME}
)


# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()

