#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

import numpy as np
import math
import time
import tf2_ros


class EKFNode(Node):

    def __init__(self):
        super().__init__("EKF_node")

        self.lidarpose_sub = self.create_subscription(
            Odometry,
            "/odom_rf2o",
            self.callback_lidar,
            10
        )
        self.imu_sub = self.create_subscription(
            Imu,
            "/imu/data_raw",
            self.callback_imu,
            10
        )

        self.pose_pub = self.create_publisher(Odometry, '/ekf_odom', 10)
        self.get_logger().info("EKF Node started.")

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Parametrii EKF 5 stari, viteza unghiulara o iau direct din imu
        self.x = np.zeros(5)
        self.P = np.diag([1.0,1.0,(np.pi / 6)**2,0.5,0.5])

        self.sigma_a = 0.02
        self.sigma_omega = 0.05
        self.R_pose = np.diag([0.0004, 0.0004, (np.pi / 180)**2])
        
        self.last_imu_time = None
        self.have_initial_pose = None
        self.ax_b_prev = None
        self.ay_b_prev = None

    def wrap(self,a):
        return (a + np.pi) % (2*np.pi) - np.pi

    # AICI SE FACE PREDICT
    def callback_imu(self, msg: Imu):
        ts = msg.header.stamp
        t = float(ts.sec) + float(ts.nanosec) * 1e-9

        if self.last_imu_time is None:
            self.last_imu_time = t
            return

        dt = t - self.last_imu_time
        if dt <= 0 or dt > 1.0:
            self.last_imu_time = t
            return

        self.last_imu_time = t

        ax_b = float(msg.linear_acceleration.x)
        ay_b = float(msg.linear_acceleration.y)
        omega_z = float(msg.angular_velocity.z)

        alpha = 0.05 # filtram un pic acceleratia
        if self.ax_b_prev == None:
            ax_b_filtered = ax_b
            self.ax_b_prev = ax_b

        else:
            ax_b_filtered = alpha * ax_b + (1 - alpha) * self.ax_b_prev
            self.ax_b_prev = ax_b

        if self.ay_b_prev == None:
            ay_b_filtered = ay_b
            self.ay_b_prev = ay_b
        else:
            ay_b_filtered = alpha * ay_b + (1 - alpha) * self.ay_b_prev
            self.ay_b_prev = ay_b
        
        self.predict(ax_b_filtered, ay_b_filtered, omega_z, dt)
    
    def predict(self, ax_b, ay_b, omega_z, dt):
        px, py, yaw, vx, vy = self.x

        cos = math.cos(yaw)
        sin = math.sin(yaw)

        ax_w = cos * ax_b - sin * ay_b # acceleratie pe axa x in raport cu sistemul de coordonate global
        ay_w = sin * ax_b + cos * ay_b # acceleratie pe axa y in raport cu sistemul de coordonate global

        px_new = px + vx * dt # x = x + vx * dt
        py_new = py + vy * dt # y = y + vy * dt
        yaw_new = yaw + omega_z * dt # yaw = yaw + omgega_z * dt
        yaw_new = self.wrap(yaw_new)
        vx_new = vx + ax_w * dt # vx = vx + (cos(yaw) * ax_b - sin(yaw) * ay_b)
        vy_new = vy + ay_w * dt # vy = vy + (sin(yaw) * ax_b + cos(yaw) * ay_b)

        x_pred = np.array([px_new, py_new, yaw_new, vx_new, vy_new]) # [x y yaw vx vy]

        F = np.eye(5)   # x' = F * x
        F[0,3] = dt
        F[1,4] = dt
        F[3,2] = -ay_w * dt
        F[4,2] = ax_w * dt

        Q = np.zeros((5,5))
        Q[3,3] = self.sigma_a**2 * dt**2 # varinata pt vx
        Q[4,4] = self.sigma_a**2 * dt**2 # varianta pt vy
        Q[0,0] = 0.25 * self.sigma_a**2 * dt**4 # varianta pt x
        Q[1,1] = 0.25 * self.sigma_a**2 * dt**4 # varianta py y
        Q[2,2] = self.sigma_omega**2 * dt**2   # varianta pt yaw

        self.x = x_pred
        self.P = F.dot(self.P).dot(F.T) + Q

    #AICI SE FACE UPDATE
    def callback_lidar(self, msg: Odometry):
        ts = msg.header.stamp
        t_msg = float(ts.sec) + float(ts.nanosec) * 1e-9

        zx = float(msg.pose.pose.position.x)
        zy = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        zyaw = math.atan2(siny_cosp, cosy_cosp)

        z = np.array([zx, zy, zyaw])

        R = None
        cov = np.array(msg.pose.covariance).reshape((6,6))
        if np.any(np.array(cov).flatten()):
            rxx = cov[0,0] if cov[0,0] > 0 else self.R_pose[0,0]
            ryy = cov[1,1] if cov[1,1] > 0 else self.R_pose[1,1]
            ryaw = cov[5,5] if cov[5,5] > 0 else self.R_pose[2,2]
            R = np.diag([rxx,ryy,ryaw])
        else:
            R = self.R_pose

        ok = self.update_pose(z,R)
        if ok:
            self.publish_ekf_odom(msg.header.stamp)
    
    def update_pose(self, z, R):
        H = np.zeros((3,5))
        H[0,0] = 1.0
        H[1,1] = 1.0
        H[2,2] = 1.0

        h = np.array([self.x[0], self.x[1], self.x[2]])
        y = z - h
        
        S = H.dot(self.P).dot(H.T) + R

        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.get_logger().error("S not invertible in update; skipping")
            return False

        K = self.P.dot(H.T).dot(S_inv)
        dx = K.dot(y)
        self.x = self.x + dx
        self.x[2] = self.wrap(self.x[2])
        
        I = np.eye(self.P.shape[0])
        self.P = (I - K.dot(H).dot(self.P).dot(I - K.dot(H)).T) + K.dot(R).dot(K.T)

        return True
    
    def publish_ekf_odom(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = float(self.x[0])
        odom.pose.pose.position.y = float(self.x[1])
        odom.pose.pose.position.z = 0.0

        yaw = float(self.x[2])
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        odom.pose.pose.orientation.w = cy
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sy

        pose_cov = np.zeros((6,6))
        pose_cov[0,0] = self.P[0,0]   # px var
        pose_cov[1,1] = self.P[1,1]   # py var
        pose_cov[5,5] = self.P[2,2]   # yaw var (mapped to index 3)
        odom.pose.covariance = list(pose_cov.flatten())

        self.pose_pub.publish(odom)

        if True:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = 'map'
            tf_msg.child_frame_id = 'base_link'
            tf_msg.transform.translation.x = float(self.x[0])
            tf_msg.transform.translation.y = float(self.x[1])
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.w = cy
            tf_msg.transform.rotation.x = 0.0
            tf_msg.transform.rotation.y = 0.0
            tf_msg.transform.rotation.z = sy
            self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"EKF shutting down.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


