#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import casadi as cas
import numpy as np
import math
import threading
from scipy.io import loadmat

from script_msgs.msg import MotorCommandObj


class NMPCNode(Node):

    def __init__(self):
        super().__init__('nmpc_node')

        qos = QoSProfile(depth=10)

        # Publisher
        self.cmd_pub = self.create_publisher(
            MotorCommandObj,
            '/motor_commands',
            qos
        )

        # Subscribers
        self.create_subscription(Odometry, '/ekf_odom', self.cb_ekf, qos)
        self.create_subscription(Imu, '/imu/data_raw', self.cb_imu, qos)

        self.get_logger().info("NMPC node started")

        # ---------------- Load reference ----------------
        data = loadmat('Trajectory_data_8.mat')
        self.Ts = float(data['Ts'])
        self.ref = data['Xref'][:, 1:]   # identical to original
        self.steps = self.ref.shape[1]

        # ---------------- MPC parameters ----------------
        self.N = 10
        self.nx = 6
        self.nu = 2

        self.alfa = 54.7
        self.beta = 54.7

        # ---------------- Internal state ----------------
        self.lock = threading.Lock()
        self.ekf_state = None
        self.imu_omega = 0.0
        self.ref_index = 0

        # ---------------- Build MPC ----------------
        self.build_dynamics()
        self.build_opti()

        # Timer
        self.timer = self.create_timer(self.Ts, self.control_loop)

        self.u_warm = np.zeros((self.nu, self.N))
        self.x_warm = np.zeros((self.nx, self.N))

    # --------------------------------------------------
    def build_dynamics(self):
        x1, x2, th, v1, v2, w = cas.MX.sym('x1'), cas.MX.sym('x2'), cas.MX.sym('th'), cas.MX.sym('v1'), cas.MX.sym('v2'), cas.MX.sym('w')
        u1, u2 = cas.MX.sym('u1'), cas.MX.sym('u2')

        x = cas.vertcat(x1, x2, th, v1, v2, w)
        u = cas.vertcat(u1, u2)

        dyn = cas.vertcat(
            v1,
            v2,
            w,
            (u1 + u2) * cas.cos(th),
            (u1 + u2) * cas.sin(th),
            self.alfa * u1 - self.beta * u2
        )

        ode = {'x': x, 'p': u, 'ode': dyn}
        self.F = cas.integrator('F', 'rk', ode, 0, self.Ts)

    # --------------------------------------------------
    def build_opti(self):
        N, nx, nu = self.N, self.nx, self.nu

        opti = cas.Opti()

        x_v = opti.variable(nx, N)
        u_v = opti.variable(nu, N)

        x_ref = opti.parameter(nx, N)
        x0 = opti.parameter(nx, 1)

        Q = np.diag([300, 300, 200, 0.1, 0.1, 0.1])
        R = np.diag([200, 200])
        P = 5 * Q

        u_bound = np.array([[2.6], [2.6]])

        obj = 0
        for i in range(N - 1):
            obj += (x_v[:, i] - x_ref[:, i]).T @ Q @ (x_v[:, i] - x_ref[:, i])
            obj += (u_v[:, i + 1] - u_v[:, i]).T @ R @ (u_v[:, i + 1] - u_v[:, i])

        obj += (x_v[:, -1] - x_ref[:, -1]).T @ P @ (x_v[:, -1] - x_ref[:, -1])
        opti.minimize(obj)

        opti.subject_to(x_v[:, 0] == self.F(x0=x0, p=u_v[:, 0])['xf'])
        for i in range(1, N):
            opti.subject_to(x_v[:, i] == self.F(x0=x_v[:, i - 1], p=u_v[:, i])['xf'])

        opti.subject_to(opti.bounded(-u_bound, u_v, u_bound))
        opti.solver('ipopt', {}, {'print_level': 0})

        self.opti = opti
        self.x_v = x_v
        self.u_v = u_v
        self.x_ref = x_ref
        self.x0 = x0

    # --------------------------------------------------
    def cb_ekf(self, msg: Odometry):
        with self.lock:
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y

            q = msg.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )

            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y

            self.ekf_state = (px, py, yaw, vx, vy)

    def cb_imu(self, msg: Imu):
        with self.lock:
            self.imu_omega = msg.angular_velocity.z
    def send_stop(self):
        cmd = MotorCommandObj()
        cmd.left_motor_power= 200
        cmd.right_motor_power=200
        cmd.miliseconds = int(500)	
        self.cmd_pub.publish(cmd)
    # --------------------------------------------------
    def control_loop(self):
        with self.lock:
            if self.ekf_state is None:
                return

            px, py, yaw, vx, vy = self.ekf_state
            omega = self.imu_omega

        if self.ref_index + self.N >= self.steps:
            self.get_logger().info("Reference finished")
            self.send_stop()
            return

        v_forward = math.hypot(vx, vy)
        v1 = v_forward * math.cos(yaw)
        v2 = v_forward * math.sin(yaw)

        x0_val = np.array([[px], [py], [yaw], [v1], [v2], [omega]])
        ref_block = self.ref[:, self.ref_index:self.ref_index + self.N]

        try:
            self.opti.set_initial(self.u_v, self.u_warm)
            self.opti.set_initial(self.x_v, self.x_warm)
            
            self.opti.set_value(self.x0, x0_val)
            self.opti.set_value(self.x_ref, ref_block)

            sol = self.opti.solve()
            u = sol.value(self.u_v)[:, 0]

            cmd = MotorCommandObj()
            if u[0] > 0:
                cmd.left_motor_power  = int(u[0] * 60 / 2.6 + 40)
            else:
                cmd.left_motor_power = int(u[0] * 60 / 2.6 - 40)
            if u[1] > 0:
                cmd.right_motor_power = int(u[1] * 60 / 2.6 + 40)
            else:
                cmd.right_motor_power = int(u[1] * 60 / 2.6 - 40)
            
            cmd.miliseconds = int(self.Ts * 1000)

            self.cmd_pub.publish(cmd)
            self.ref_index += 1

            self.u_warm = sol.value(self.u_v)
            self.x_warm = sol.value(self.x_v)

        except Exception as e:
            self.get_logger().error(f"NMPC failed: {e}")


def main():
    rclpy.init()
    node = NMPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
