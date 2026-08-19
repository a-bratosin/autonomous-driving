#!/usr/bin/env python3
"""
mpc_node.py

Online MPC node:
 - Loads Reference.mat and builds xref the same way as your MATLAB script.
 - Subscribes to /ekf_odom (nav_msgs/Odometry) and /imu/data_raw (sensor_msgs/Imu).
 - Uses EKF-measured state as x0 and runs CasADi Opti MPC (identical structure to your MATLAB code).
 - Publishes first control to /mpc_commands (std_msgs/Float64MultiArray).
 - Publishes predicted states to /mpc_predicted_odom (nav_msgs/Odometry) for visualization.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from communication_interfaces.msg import MotorCommandObj

import numpy as np
import math
import threading
import time

import casadi as ca

# scipy utilities for Reference.mat processing
try:
    from scipy.io import loadmat
    from scipy.interpolate import interp1d
    from scipy.ndimage import uniform_filter1d
    SCIPY_AVAILABLE = True
except Exception:
    SCIPY_AVAILABLE = False


def big_diag_m(M, N):
    """Manual block-diagonal for CasADi (works in all versions)."""
    # Accept either numpy arrays or CasADi DM
    if isinstance(M, np.ndarray):
        rows, cols = M.shape
        Z = ca.DM.zeros(rows * N, cols * N)
        for i in range(N):
            Z[i*rows:(i+1)*rows, i*cols:(i+1)*cols] = ca.DM(M)
        return Z
    else:
        # assume CasADi DM or SX/MX converted to DM
        Md = ca.DM(M)
        rows, cols = Md.shape
        Z = ca.DM.zeros(rows * N, cols * N)
        for i in range(N):
            Z[i*rows:(i+1)*rows, i*cols:(i+1)*cols] = Md
        return Z


class MPCNode(Node):
    def __init__(self):
        super().__init__("mpc_node")

        qos = QoSProfile(depth=10)

        # Publishers
        self.u_pub = self.create_publisher(MotorCommandObj, "/motor_commands", qos)
        self.pred_pub = self.create_publisher(Odometry, "/mpc_predicted_odom", qos)

        # Subscribers
        self.ekf_sub = self.create_subscription(Odometry, "/ekf_odom", self.cb_ekf, qos)
        self.imu_sub = self.create_subscription(Imu, "/imu/data_raw", self.cb_imu, qos)

        self.get_logger().info("MPC Node starting...")

        # ---------------- MPC parameters (matching MATLAB, with alfa=beta requested) ----------------
        self.alfa = 0.1
        self.beta = 0.1
        self.nx = 6
        self.nu = 2
        self.Ts = 0.1
        self.m_mul = 1
        self.N = 20

        # cost matrices same as MATLAB
        Q_base = np.diag([5000, 5000, 5000, 5000, 5000, 5000])
        R_base = np.diag([0.1, 0.1])
        self.Q_big = big_diag_m(Q_base, self.N)
        self.R_big = big_diag_m(R_base, self.N)

        # storage for EKF & IMU
        self.lock = threading.Lock()
        self.last_ekf_msg = None          # latest Odometry message
        self.last_ekf_time = None        # python float seconds
        self.prev_pose = None            # for finite-diff velocity
        self.prev_pose_time = None
        self.latest_imu_omega = None

        # Build model + opti (CasADi)
        self.build_casadi_model()
        self.build_opti_problem()

        # Load reference from Reference.mat and construct xref like MATLAB
        if not SCIPY_AVAILABLE:
            self.get_logger().error("scipy required to load Reference.mat. Please install scipy.")
            raise RuntimeError("scipy not available")
        try:
            self.Xref = self.load_reference_mat("Reference.mat")
            self.get_logger().info("Loaded Reference.mat successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load Reference.mat: {e}")
            raise

        self.xref_sequence = self.build_xref_sequence(self.Xref, self.m_mul)

        # Keep track of reference index (MATLAB loop moves index by 1 each iteration)
        self.ref_index = 0
        self.ref_length = (self.xref_sequence.size // self.nx)
        self.get_logger().info(f"Reference sequence length (steps): {self.ref_length}")

        # Variables to warm-start solver
        self.last_sol_x = np.zeros(self.nx * self.N)
        self.last_sol_u = np.zeros(self.nu * self.N)

        # default x_init (will be updated from EKF)
        self.x_init = np.zeros(self.nx)

        # Start timer with Ts
        self.timer = self.create_timer(self.Ts, self.on_timer)

    # ----------------------- Reference.mat loading & preprocessing -----------------------
    def load_reference_mat(self, path):
        """Load Reference.mat and return Xref (expects variable named Xref)."""
        mat = loadmat(path)
        if "Xref" not in mat:
            raise ValueError("Reference.mat does not contain 'Xref' variable")
        Xref = mat["Xref"]
        # Ensure shape is (m, n) with MATLAB-style orientation
        return Xref

    def build_xref_sequence(self, Xref, m_mul=1):
        """
        Reproduce MATLAB preprocessing:
        - For each column of Xref, interpolate with pchip (cubic) to m_mul*length(vec)
        - smoothdata(vec,'movmean',m_mul)
        - Build Xref_new as rows stacked, then xref = [Xref_new(2,:); Xref_new(3,:); ...] flattened by column
        """
        m, n = Xref.shape  # MATLAB: [m,n] = size(Xref)
        Xref_new_cols = []

        for j in range(n):
            vec = Xref[:, j]
            t = np.arange(1, len(vec) + 1)
            ti = np.linspace(1, len(vec), m_mul * len(vec))
            # interp1(...,'pchip') -> use cubic interp1d
            f = interp1d(t, vec, kind="cubic")
            veci = f(ti)
            # smoothdata with movmean (window m_mul)
            if m_mul > 1:
                veci = uniform_filter1d(veci, size=m_mul, mode="nearest")
            Xref_new_cols.append(veci)

        # Xref_new is stacked rows (MATLAB transposes later)
        Xref_new = np.array(Xref_new_cols).T  # shape (m*m_mul, n)
        m2, n2 = Xref_new.shape

        # Build xref as MATLAB did: for i = 2:m, xref = [xref Xref_new(i,:)]
        # So m2 corresponds to number of time samples; they start from i=2 to m2 (1-based)
        xref_list = []
        for i in range(1, m2):  # skip first row (i=1 in MATLAB)
            row = Xref_new[i, :]
            xref_list.extend(row.tolist())

        xref = np.array(xref_list).reshape((-1, 1))  # column vector
        return xref

    # ----------------------- CasADi model (same ODE as MATLAB) -----------------------
    def build_casadi_model(self):
        nx = self.nx
        nu = self.nu

        x_sym = ca.MX.sym("x", nx)
        u_sym = ca.MX.sym("u", nu)

        x1 = x_sym[0]
        x2 = x_sym[1]
        teta = x_sym[2]
        v1 = x_sym[3]
        v2 = x_sym[4]
        omega = x_sym[5]
        u1 = u_sym[0]
        u2 = u_sym[1]

        # ode = [v1;v2;omega;(u1+u2)*cos(teta);(u1+u2)*sin(teta);alfa*u1-beta*u2];
        ode = ca.vertcat(
            v1,
            v2,
            omega,
            (u1 + u2) * ca.cos(teta),
            (u1 + u2) * ca.sin(teta),
            self.alfa * u1 - self.beta * u2,
        )

        dae = {"x": x_sym, "p": u_sym, "ode": ode}
        opts = {"tf": self.Ts / self.m_mul, "simplify": True, "number_of_finite_elements": 1}
        integrator = ca.integrator("intg", "rk", dae, opts)
        res = integrator(x0=x_sym, p=u_sym)
        x_next = res["xf"]
        self.F = ca.Function("F", [x_sym, u_sym], [x_next], ["x", "u"], ["x_next"])
        self.intg = integrator

    # ----------------------- CasADi Opti problem (match MATLAB) -----------------------
    def build_opti_problem(self):
        nx = self.nx
        nu = self.nu
        N = self.N

        opti = ca.Opti()
        x = opti.variable(nx * N, 1)
        u = opti.variable(nu * N, 1)

        # Create Opti parameters (so we can set their values via opti.set_value)
        x_ref = opti.parameter(nx * N, 1)
        x0 = opti.parameter(nx, 1)

        # cost: x'*Q*x - 2*x_ref'*Q*x + u'*R*u
        cost = ca.mtimes([x.T, self.Q_big, x]) - 2 * ca.mtimes([x_ref.T, self.Q_big, x]) + ca.mtimes([u.T, self.R_big, u])
        opti.minimize(cost)

        # dynamics constraints
        # first block
        opti.subject_to(x[0:nx] == self.F(x0, u[0:nu]))
        # rest
        for k in range(1, N):
            idx_prev = k - 1
            x_prev = x[idx_prev * nx: (idx_prev + 1) * nx]
            x_curr = x[k * nx: (k + 1) * nx]
            u_k = u[k * nu: (k + 1) * nu]
            opti.subject_to(x_curr == self.F(x_prev, u_k))

        # bounds -1 <= u <= 1
        opti.subject_to(opti.bounded(-1, u, 1))

        opti.solver("ipopt", {}, {"print_level": 0, "max_iter": 1000})
        self.opti = opti
        self.opti_x = x
        self.opti_u = u
        # store parameters (MX objects) but use opti.set_value to update them
        self.opti_xref = x_ref
        self.opti_x0 = x0

    # ----------------------- EKF / IMU callbacks -----------------------
    def cb_ekf(self, msg: Odometry):
        """Store EKF pose; compute vx,vy by finite-difference if twist not provided."""
        with self.lock:
            t_ros = msg.header.stamp
            t = float(t_ros.sec) + float(t_ros.nanosec) * 1e-9

            px = float(msg.pose.pose.position.x)
            py = float(msg.pose.pose.position.y)

            # orientation -> yaw
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            # try twist
            vx = 0.0
            vy = 0.0
            used_twist = False
            try:
                vx = float(msg.twist.twist.linear.x)
                vy = float(msg.twist.twist.linear.y)
                used_twist = True
            except Exception:
                used_twist = False

            # if twist is zero or not present, estimate via finite difference
            if not used_twist:
                if (self.prev_pose is not None) and (self.prev_pose_time is not None):
                    dt = t - self.prev_pose_time
                    if dt > 1e-6 and dt < 1.0:
                        vx = (px - self.prev_pose[0]) / dt
                        vy = (py - self.prev_pose[1]) / dt
                    else:
                        vx = 0.0
                        vy = 0.0
                else:
                    vx = 0.0
                    vy = 0.0

            # store prev pose/time for next finite diff
            self.prev_pose = (px, py)
            self.prev_pose_time = t

            self.last_ekf_msg = {"px": px, "py": py, "yaw": yaw, "vx": vx, "vy": vy, "t": t}

    def cb_imu(self, msg: Imu):
        with self.lock:
            omega_z = float(msg.angular_velocity.z)
            self.latest_imu_omega = {"omega_z": omega_z, "t": float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9}

    # ----------------------- MPC timer: use EKF x0 and Reference -----------------------
    def on_timer(self):
        # get latest ekf + imu
        with self.lock:
            ekf = None if self.last_ekf_msg is None else dict(self.last_ekf_msg)
            imu = None if self.latest_imu_omega is None else dict(self.latest_imu_omega)

        if ekf is None:
            # wait for EKF
            return

        # compose x0 = [x1; x2; teta; v1; v2; omega]
        px = ekf["px"]
        py = ekf["py"]
        yaw = ekf["yaw"]
        vx = ekf["vx"]
        vy = ekf["vy"]

        # use imu omega if available, else set 0
        omega_z = imu["omega_z"] if imu is not None else 0.0

        # differential-drive mapping requested:
        # v_forward = sqrt(vx^2 + vy^2)
        # v1 = v_forward * cos(yaw)
        # v2 = v_forward * sin(yaw)
        v_forward = math.hypot(vx, vy)
        v1 = v_forward * math.cos(yaw)
        v2 = v_forward * math.sin(yaw)

        x0_np = np.array([px, py, yaw, v1, v2, omega_z], dtype=float).reshape((self.nx,))

        # build x_ref block for current reference index
        # MATLAB used: for k iteration it took xref = [xref(1+(k-1)*nx : N*nx+(k-1)*nx)]
        # Our ref_index corresponds to MATLAB k-1
        # Check bounds
        if self.ref_index + self.N > self.ref_length:
            # no more reference window to follow
            self.get_logger().info("Reference exhausted or too short for horizon. Holding last command.")
            # optionally still run MPC with repeated last reference point
            # create repeated target of last ref
            last_ref_idx = max(0, self.ref_length - 1)
            ref_start = last_ref_idx * self.nx
            single_ref = self.xref_sequence[ref_start: ref_start + self.nx].flatten()
            xref_block = np.tile(single_ref.reshape((-1, 1)), (self.N, 1))
        else:
            ref_start = self.ref_index * self.nx
            # slice of length nx*N
            xref_block = self.xref_sequence[ref_start: ref_start + self.nx * self.N].reshape((-1, 1))

        # solve MPC
        try:
            # set parameters USING opti.set_value (NOT param.set_value)
            self.opti.set_value(self.opti_x0, ca.DM(x0_np))
            self.opti.set_value(self.opti_xref, ca.DM(xref_block))

            # warm-start
            try:
                self.opti.set_initial(self.opti_x, ca.DM(self.last_sol_x))
                self.opti.set_initial(self.opti_u, ca.DM(self.last_sol_u))
            except Exception:
                pass

            sol = self.opti.solve()

            sol_x = np.array(sol.value(self.opti_x)).flatten()
            sol_u = np.array(sol.value(self.opti_u)).flatten()

            # store warm-start
            self.last_sol_x = sol_x
            self.last_sol_u = sol_u

            # first control action
            u0 = sol_u[0:self.nu]

            # publish control
            cmd = MotorCommandObj()
            cmd.left_motor_power  = int(np.clip(u0[0] * 100, -100, 100))   # scale MPC output
            cmd.right_motor_power = int(np.clip(u0[1] * 100, -100, 100))
            cmd.miliseconds = int(self.Ts * 1000)

            self.u_pub.publish(cmd)

            # publish predicted trajectory for visualization
            for k in range(self.N):
                idx = k * self.nx
                st = sol_x[idx: idx + self.nx]
                self.publish_predicted(k, st)

            # update index (MATLAB loop advanced by 1 each iteration)
            self.ref_index += 1

        except Exception as e:
            self.get_logger().error(f"MPC solve failed: {e}")

    # ----------------------- publish predicted states -----------------------
    def publish_predicted(self, k, st):
        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = "map"
        od.child_frame_id = f"mpc_pred_{k}"

        od.pose.pose.position.x = float(st[0])
        od.pose.pose.position.y = float(st[1])
        yaw = float(st[2])
        cy = math.cos(0.5 * yaw)
        sy = math.sin(0.5 * yaw)
        od.pose.pose.orientation.w = cy
        od.pose.pose.orientation.x = 0.0
        od.pose.pose.orientation.y = 0.0
        od.pose.pose.orientation.z = sy

        self.pred_pub.publish(od)


def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down MPC node.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
