#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt


class DebugPlotter(Node):
    def __init__(self):
        super().__init__('debug_plotter')
        
        self.actual_x, self.actual_y = [], []
        self.ref_x, self.ref_y = [], []

        self.create_subscription(Pose, '/curr_pose', self.actual_cb, 10)
        self.create_subscription(Pose, '/ref_pose', self.ref_cb, 10)

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.line_actual, = self.ax.plot([], [], 'r-', label='Actual Path')
        self.line_ref, = self.ax.plot([], [], 'b--', label='Reference Path')
        
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_title('NMPC Trajectory Tracking')
        self.ax.legend()
        self.ax.grid(True)

        self.get_logger().info("Debug Plotter initialized.")

    def actual_cb(self, msg: Pose):
        self.actual_x.append(msg.position.x)
        self.actual_y.append(msg.position.y)

    def ref_cb(self, msg: Pose):
        self.ref_x.append(msg.position.x)
        self.ref_y.append(msg.position.y)

    def update_plot(self):
        # Desenează dacă avem măcar un punct în oricare dintre trasee
        if not self.actual_x and not self.ref_x:
            return

        if not plt.fignum_exists(self.fig.number):
            return

        if self.actual_x:
            self.line_actual.set_data(self.actual_x, self.actual_y)
        if self.ref_x:
            self.line_ref.set_data(self.ref_x, self.ref_y)

        self.ax.relim()
        self.ax.autoscale_view()
        
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = DebugPlotter()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.update_plot()
            plt.pause(0.05)  # Update la ~20 FPS pentru randare optimă
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
if __name__ == "__main__":
    main()
