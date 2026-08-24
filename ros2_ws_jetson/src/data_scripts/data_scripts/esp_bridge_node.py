#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from script_msgs.msg import MotorCommandObj
import serial

class EspBridgeNode(Node):
    def __init__(self):
        super().__init__('esp_bridge_node')
        
        # Parametri pentru portul serial (poti schimba in functie de cum vede Jetson-ul ESP-ul)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.01)
            self.get_logger().info(f"Conectat cu succes la ESP32 pe {port} la {baudrate} baud.")
        except Exception as e:
            self.get_logger().error(f"Eroare la deschiderea portului serial {port}: {e}")
            self.ser = None

        # Publisher pentru datele IMU venite de la ESP32
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        
        # Subscriber pentru comenzile de motoare trimise de MPC/Control
        self.motor_sub = self.create_subscription(
            MotorCommandObj,
            '/motor_commands',
            self.motor_callback,
            10
        )
        
        # Timer rapid pentru a citi in continuu datele venite de la ESP32 pe Serial (100Hz)
        self.timer = self.create_timer(0.01, self.serial_read_callback)

    def motor_callback(self, msg):
        if self.ser and self.ser.is_open:
            # Format trimis la ESP32: M:stanga,dreapta,timp\n
            command_str = f"M:{msg.left_motor_power},{msg.right_motor_power},{msg.miliseconds}\n"
            try:
                self.ser.write(command_str.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Eroare la scriere seriala: {e}")

    def serial_read_callback(self):
        if not self.ser or not self.ser.is_open:
            return
        
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("IMU:"):
                    data = line[4:].split(',')
                    if len(data) == 6:
                        imu_msg = Imu()
                        imu_msg.header.stamp = self.get_clock().now().to_msg()
                        imu_msg.header.frame_id = "imu_link"
                        
                        # Acceleratie liniara
                        imu_msg.linear_acceleration.x = float(data[0])
                        imu_msg.linear_acceleration.y = float(data[1])
                        imu_msg.linear_acceleration.z = float(data[2])
                        
                        # Viteza unghiulara
                        imu_msg.angular_velocity.x = float(data[3])
                        imu_msg.angular_velocity.y = float(data[4])
                        imu_msg.angular_velocity.z = float(data[5])
                        
                        # Covarianta -1 indica orientare necunoscuta (date brute)
                        imu_msg.orientation_covariance[0] = -1.0
                        
                        self.imu_pub.publish(imu_msg)
        except Exception as e:
            self.get_logger().error(f"Eroare la citirea de pe serial: {e}")

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EspBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
