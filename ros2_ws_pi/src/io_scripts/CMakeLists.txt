cmake_minimum_required(VERSION 3.8)
project(io_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommandObj.msg"
)

# Install Python scripts
install(PROGRAMS
  scripts/motor_controller.py
  scripts/imu_publisher.py
  scripts/imu_reader.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
