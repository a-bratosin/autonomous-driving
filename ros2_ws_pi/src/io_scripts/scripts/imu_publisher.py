#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# Import the IMU reader script directly
import imu_reader

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        
        # Publisher for raw IMU data
        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)
        
        # Timer set to 10Hz (0.1 seconds)
        timer_period = 0.1 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        try:
            self.bus = imu_reader.init_imu()
            self.get_logger().info("IMU initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize IMU: {e}")
            raise e

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        try:
            # Call the function from imu_reader.py
            accel, gyro = imu_reader.read_sensor_data(self.bus)
            
            # Map Angular velocity (rad/s)
            msg.angular_velocity.x = gyro[0]
            msg.angular_velocity.y = gyro[1]
            msg.angular_velocity.z = gyro[2]
            
            # Map Linear acceleration (m/s^2)
            msg.linear_acceleration.x = accel[0]
            msg.linear_acceleration.y = accel[1]
            msg.linear_acceleration.z = accel[2]
            
            # Set covariance to -1 to indicate orientation is unknown (raw data only)
            msg.orientation_covariance[0] = -1.0 
            
            self.publisher_.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error reading IMU data: {e}")

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()
    
    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
