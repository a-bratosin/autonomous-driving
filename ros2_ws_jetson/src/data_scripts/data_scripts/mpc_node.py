#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose

import casadi as cas
import numpy as np
import math
import threading
from scipy.io import loadmat

# Asigura-te ca pachetul se numeste script_msgs. Daca e scripts_msgs, modifica aici!
from script_msgs.msg import MotorCommandObj
import time


class NMPCNode(Node):

    def __init__(self):
        super().__init__('nmpc_node')

        qos = QoSProfile(depth=10)

        # Publishers
        self.cmd_pub = self.create_publisher(MotorCommandObj, '/motor_commands', qos)
        self.curr_pose_pub = self.create_publisher(Pose, '/curr_pose', qos)
        self.ref_pose_pub = self.create_publisher(Pose, '/ref_pose', qos)

        # Subscribers
        self.create_subscription(Odometry, '/ekf_odom', self.cb_ekf, qos)
        self.create_subscription(Imu, '/imu/data_raw', self.cb_imu, qos)

        # ---------------- Cautare automata Trajectory_data_8.mat ----------------
        mat_filename = 'Trajectory_data_8.mat'
        possible_paths = [
            mat_filename,
            os.path.join(os.getcwd(), mat_filename),
            os.path.join('/root/ros2_ws_jetson', mat_filename),
            os.path.join(os.path.dirname(__file__), '..', '..', '..', mat_filename)
        ]
        mat_path = None
        for path in possible_paths:
            if os.path.exists(path):
                mat_path = path
                break

        if mat_path is None:
            self.get_logger().fatal(f"Could not find {mat_filename} in any search path!")
            raise FileNotFoundError(f"{mat_filename} not found.")

        self.get_logger().info(f"Loading reference trajectory from: {mat_path}")
        data = loadmat(mat_path)
        self.Ts = float(data['Ts'])
        self.ref = data['x_aux'][:, 1:]
        self.steps = self.ref.shape[1]

        # ---------------- MPC parameters ----------------
        self.N = 4
        self.nx = 6
        self.nu = 2
        self.alfa = 54.7
        self.beta = 54.7
        self.u_max = 2.6
        self.prag_pwm = 20  # Prag minim PWM in procentaje

        # ---------------- Internal state ----------------
        self.lock = threading.Lock()
        self.ekf_state = None
        self.imu_omega = 0.0
        self.ref_index = 0

        # ---------------- Build MPC ----------------
        self.build_dynamics()
        self.build_opti()

        self.u_warm = np.zeros((self.nu, self.N))
        self.x_warm = np.zeros((self.nx, self.N))

        # Timer bucla de control
        self.timer = self.create_timer(self.Ts, self.control_loop)
        self.get_logger().info("NMPC node initialized and ready.")

    def build_dynamics(self):
        x1, x2, th = cas.MX.sym('x1'), cas.MX.sym('x2'), cas.MX.sym('th')
        v1, v2, w = cas.MX.sym('v1'), cas.MX.sym('v2'), cas.MX.sym('w')
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

    def build_opti(self):
        N, nx, nu = self.N, self.nx, self.nu
        opti = cas.Opti()

        x_v = opti.variable(nx, N)
        u_v = opti.variable(nu, N)

        x_ref = opti.parameter(nx, N)
        x0 = opti.parameter(nx, 1)

        Q = np.diag([500, 500, 1000, 100, 100, 1000])
        R = np.diag([10, 10])
        P = 1 * Q

        u_bound = np.array([[self.u_max], [self.u_max]])

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
        
        # CORECTIE: Optiuni solver silentios
        opts = {'expand': True, 'print_time': False}
        ipopt_opts = {'print_level': 0, 'sb': 'yes'}
        opti.solver('ipopt', opts, ipopt_opts)

        self.opti = opti
        self.x_v = x_v
        self.u_v = u_v
        self.x_ref = x_ref
        self.x0 = x0

    # CORECTIE: Functie de deadband curata
    def scale_to_pwm(self, u_val):
        """Converteaza valoarea comenzii u (-2.6..2.6) in PWM (-100..100) cu zona moarta (prag)."""
        if abs(u_val) < 1e-4:
            return 0
        
        # Procentaj baza [0..1]
        ratio = min(abs(u_val) / self.u_max, 1.0)
        # Scalare luand in calcul pragul minim (ex: de la 20% la 100%)
        pwm = self.prag_pwm + ratio * (100 - self.prag_pwm)
        
        return int(math.copysign(pwm, u_val))

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
        cmd.left_motor_power = 0
        cmd.right_motor_power = 0
        cmd.miliseconds = int(500)
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        with self.lock:
            if self.ekf_state is None:
                return
            px, py, yaw, vx, vy = self.ekf_state
            omega = self.imu_omega

        # CORECTIE: Prevenirea depasirii limitelor matricii de referinta
        if self.ref_index + self.N >= self.steps:
            self.get_logger().info("Trajectory reference finished.")
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

            init_time = time.perf_counter()
            sol = self.opti.solve()
            final_time = time.perf_counter()

            u = sol.value(self.u_v)[:, 0]

            cmd = MotorCommandObj()
            cmd.left_motor_power = self.scale_to_pwm(u[0])
            cmd.right_motor_power = self.scale_to_pwm(u[1])
            cmd.miliseconds = int(self.Ts * 1000)

            self.cmd_pub.publish(cmd)
            self.ref_index += 1

            self.u_warm = sol.value(self.u_v)
            self.x_warm = sol.value(self.x_v)

            curr_msg = Pose()
            curr_msg.position.x = float(px)
            curr_msg.position.y = float(py)
            curr_msg.orientation.z = math.sin(yaw / 2.0)
            curr_msg.orientation.w = math.cos(yaw / 2.0)
            self.curr_pose_pub.publish(curr_msg)

            ref_msg = Pose()
            ref_msg.position.x = float(ref_block[0, 0])
            ref_msg.position.y = float(ref_block[1, 0])
            ref_yaw = float(ref_block[2, 0])
            ref_msg.orientation.z = math.sin(ref_yaw / 2.0)
            ref_msg.orientation.w = math.cos(ref_yaw / 2.0)
            self.ref_pose_pub.publish(ref_msg)

        except Exception as e:
            self.get_logger().error(f"NMPC solve failed: {e}")
            self.send_stop()


def main(args=None):
    rclpy.init(args=args)
    node = NMPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
