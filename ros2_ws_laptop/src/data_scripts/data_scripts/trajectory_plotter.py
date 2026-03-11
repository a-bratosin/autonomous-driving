#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import threading


class TrajectoryPlotter(Node):

    def __init__(self):
        super().__init__("trajectory_plotter")

        # Store path
        self.x_list = []
        self.y_list = []

        # ROS subscriber
        self.sub = self.create_subscription(
            Odometry,
            "/ekf_odom",
            self.odom_callback,
            10
        )

        # Set up matplotlib live plotting in a background thread
        self.plot_thread = threading.Thread(target=self.run_plot, daemon=True)
        self.plot_thread.start()

        self.get_logger().info("Trajectory Plotter started.")

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.x_list.append(x)
        self.y_list.append(y)

    def run_plot(self):
        plt.ion()
        fig, ax = plt.subplots()
        line, = ax.plot([], [], "-b")

        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_title("EKF Estimated Trajectory")
        ax.grid(True)

        while True:
            if len(self.x_list) > 1:
                line.set_xdata(self.x_list)
                line.set_ydata(self.y_list)
                ax.relim()
                ax.autoscale()
                fig.canvas.draw()
                fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
