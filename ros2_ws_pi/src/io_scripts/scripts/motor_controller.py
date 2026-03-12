#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import lgpio

# Import the custom message from this package
from script_msgs.msg import MotorCommandObj

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('control_motoare')
        
        self.declare_parameter('pin_ena', 18)
        self.declare_parameter('pin_in1', 23)
        self.declare_parameter('pin_in2', 24)
        self.declare_parameter('pin_in3', 6)
        self.declare_parameter('pin_in4', 5)
        self.declare_parameter('pin_enb', 19)

        self.ENA = self.get_parameter('pin_ena').value
        self.IN1 = self.get_parameter('pin_in1').value
        self.IN2 = self.get_parameter('pin_in2').value
        self.IN3 = self.get_parameter('pin_in3').value
        self.IN4 = self.get_parameter('pin_in4').value
        self.ENB = self.get_parameter('pin_enb').value

        try:
            self.h = lgpio.gpiochip_open(4)
        except Exception:
            self.h = lgpio.gpiochip_open(0)

        self.setup_gpio()

        self.subscription = self.create_subscription(
            MotorCommandObj,
            '/motor_commands',
            self.command_callback,
            10
        )

        self.stop_timer = None

    def setup_gpio(self):
        pins = [self.IN1, self.IN2, self.IN3, self.IN4]
        for pin in pins:
            lgpio.gpio_claim_output(self.h, pin)

        lgpio.tx_pwm(self.h, self.ENA, 1000, 0)
        lgpio.tx_pwm(self.h, self.ENB, 1000, 0)
        self.get_logger().info("GPIO initialized successfully.")

    def stop(self):
        lgpio.gpio_write(self.h, self.IN1, 0)
        lgpio.gpio_write(self.h, self.IN2, 0)
        lgpio.gpio_write(self.h, self.IN3, 0)
        lgpio.gpio_write(self.h, self.IN4, 0)
        lgpio.tx_pwm(self.h, self.ENA, 1000, 0)
        lgpio.tx_pwm(self.h, self.ENB, 1000, 0)

    def forward_left(self, speed):
        lgpio.gpio_write(self.h, self.IN1, 1)
        lgpio.gpio_write(self.h, self.IN2, 0)
        lgpio.tx_pwm(self.h, self.ENA, 1000, speed)

    def backward_left(self, speed):
        lgpio.gpio_write(self.h, self.IN1, 0)
        lgpio.gpio_write(self.h, self.IN2, 1)
        lgpio.tx_pwm(self.h, self.ENA, 1000, speed)

    def forward_right(self, speed):
        lgpio.gpio_write(self.h, self.IN3, 1)
        lgpio.gpio_write(self.h, self.IN4, 0)
        lgpio.tx_pwm(self.h, self.ENB, 1000, speed)

    def backward_right(self, speed):
        lgpio.gpio_write(self.h, self.IN3, 0)
        lgpio.gpio_write(self.h, self.IN4, 1)
        lgpio.tx_pwm(self.h, self.ENB, 1000, speed)

    def command_callback(self, msg):
        self.get_logger().info(f"Cmd: L:{msg.left_motor_power} R:{msg.right_motor_power} T:{msg.miliseconds}ms")

        if msg.left_motor_power < 0:
            self.backward_left(abs(msg.left_motor_power))
        else:
            self.forward_left(abs(msg.left_motor_power))
        
        if msg.right_motor_power < 0:
            self.backward_right(abs(msg.right_motor_power))
        else:
            self.forward_right(abs(msg.right_motor_power))

        # Handle timing non-blockingly
        if self.stop_timer is not None:
            self.stop_timer.cancel()

        if msg.miliseconds > 0:
            timer_period = msg.miliseconds / 1000.0
            self.stop_timer = self.create_timer(timer_period, self.timer_stop_callback)

    def timer_stop_callback(self):
        self.stop()
        if self.stop_timer is not None:
            self.stop_timer.cancel()
            self.stop_timer = None

    def cleanup(self):
        self.get_logger().info("Shutting down motor controller...")
        self.stop()
        lgpio.gpiochip_close(self.h)

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
