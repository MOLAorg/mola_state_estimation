#!/usr/bin/env python3
# Synthetic sensor publisher for mola_state_estimation_smoother integration tests.
#
# Publishes noisy odometry, GNSS, and IMU messages that match a closed-form
# ground-truth trajectory, plus ground_truth/pose for the checker to consume.
#
# Run as a standalone ROS 2 node:
#   python3 sensor_mock_node.py --ros-args -p scenario:=static -p seed:=42
from common import (
    GT_LAT0_DEG, GT_LON0_DEG, GT_ALT0_M,
    enu_to_latlon, imu_quat_from_enu_pose, imu_linear_acc_body,
    moving_gt_pose, moving_gt_twist, moving_gt_world_accel,
    static_gt_pose, quaternion_from_euler_zyx,
    MOVING_DURATION_S,
)
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

_TRANSIENT_LOCAL_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


class SensorMockNode(Node):
    def __init__(self):
        super().__init__('sensor_mock')

        self.declare_parameter('scenario', 'static')
        self.declare_parameter('seed', 42)
        self.declare_parameter('startup_delay_sec', 3.0)

        # Rates [Hz]
        self.declare_parameter('odom_rate', 10.0)
        self.declare_parameter('gnss_rate', 2.0)
        self.declare_parameter('imu_rate', 20.0)

        # Noise sigmas
        self.declare_parameter('gnss_xy_sigma_m', 1.0)
        self.declare_parameter('gnss_z_sigma_m', 2.0)
        self.declare_parameter('imu_yaw_sigma_rad', math.radians(2.0))
        self.declare_parameter('imu_acc_sigma', 0.2)
        self.declare_parameter('odom_lin_sigma', 0.02)
        self.declare_parameter('odom_ang_sigma', math.radians(0.5))

        # Frame names
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('imu_frame', 'imu')
        self.declare_parameter('gnss_frame', 'gnss')

        self._scenario = self.get_parameter('scenario').value
        seed = self.get_parameter('seed').value
        self._rng = np.random.default_rng(seed)
        self._startup_delay = self.get_parameter('startup_delay_sec').value

        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_link = self.get_parameter('base_link_frame').value
        self._imu_frame = self.get_parameter('imu_frame').value
        self._gnss_frame = self.get_parameter('gnss_frame').value

        self._gnss_xy_sigma = self.get_parameter('gnss_xy_sigma_m').value
        self._gnss_z_sigma = self.get_parameter('gnss_z_sigma_m').value
        self._imu_yaw_sigma = self.get_parameter('imu_yaw_sigma_rad').value
        self._imu_acc_sigma = self.get_parameter('imu_acc_sigma').value
        self._odom_lin_sigma = self.get_parameter('odom_lin_sigma').value
        self._odom_ang_sigma = self.get_parameter('odom_ang_sigma').value

        # Publishers
        self._pub_odom = self.create_publisher(Odometry, '/wheel_odom', 10)
        self._pub_gnss = self.create_publisher(NavSatFix, '/gps', 10)
        self._pub_imu = self.create_publisher(Imu, '/imu', 10)
        self._pub_gt = self.create_publisher(
            PoseStamped, '/ground_truth/pose', 10)
        self._pub_tf_static = self.create_publisher(TFMessage, '/tf_static',
                                                    _TRANSIENT_LOCAL_QOS)

        # Accumulated dead-reckoning pose in odom frame (x, y, yaw)
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0

        self._t_start = None  # wall time when publishing first started

        odom_dt = 1.0 / self.get_parameter('odom_rate').value
        gnss_dt = 1.0 / self.get_parameter('gnss_rate').value
        imu_dt = 1.0 / self.get_parameter('imu_rate').value

        self.create_timer(odom_dt, self._odom_cb)
        self.create_timer(gnss_dt, self._gnss_cb)
        self.create_timer(imu_dt, self._imu_cb)
        # Re-publish tf_static periodically so late subscribers get it
        self.create_timer(5.0, self._publish_tf_static)

        # Publish tf_static immediately
        self._publish_tf_static()
        self.get_logger().info(
            f"SensorMockNode started: scenario={self._scenario}, seed={seed}, "
            f"startup_delay={self._startup_delay} s"
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _elapsed(self):
        """Seconds since first sensor message (None if still in delay)."""
        if self._t_start is None:
            return None
        return time.monotonic() - self._t_start

    def _maybe_start(self):
        """Return True if past the startup delay, starting the clock if needed."""
        now = time.monotonic()
        if self._t_start is None:
            if not hasattr(self, '_t_node_start'):
                self._t_node_start = now
            if now - self._t_node_start < self._startup_delay:
                return False
            self._t_start = now
            self.get_logger().info("Sensor mock: starting publication")
        return True

    def _gt_at(self, t):
        """Return (x, y, z, yaw, pitch, roll) at trajectory time t."""
        if self._scenario == 'static':
            return static_gt_pose()
        return moving_gt_pose(t)

    def _gt_twist_at(self, t):
        """Return body-frame twist (vx, vy, vz, wx, wy, wz) at trajectory time t."""
        if self._scenario == 'static':
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        return moving_gt_twist(t)

    def _now_stamp(self):
        return self.get_clock().now().to_msg()

    # ------------------------------------------------------------------
    # Static TF: base_link -> imu / gnss (identity, TRANSIENT_LOCAL)
    # ------------------------------------------------------------------

    def _publish_tf_static(self):
        now = self._now_stamp()
        msgs = []
        for child in (self._imu_frame, self._gnss_frame):
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self._base_link
            tf.child_frame_id = child
            tf.transform.rotation.w = 1.0
            msgs.append(tf)
        self._pub_tf_static.publish(TFMessage(transforms=msgs))

    # ------------------------------------------------------------------
    # Odometry callback
    # ------------------------------------------------------------------

    def _odom_cb(self):
        if not self._maybe_start():
            return

        t = self._elapsed()
        x, y, _z, yaw, _pitch, _roll = self._gt_at(t)
        vx, vy, vz, _wx, _wy, wz = self._gt_twist_at(t)

        # Integrate noisy twist into odom pose
        odom_dt = 1.0 / self.get_parameter('odom_rate').value
        noisy_vx = vx + self._rng.normal(0, self._odom_lin_sigma)
        noisy_wz = wz + self._rng.normal(0, self._odom_ang_sigma)

        self._odom_x += (noisy_vx * math.cos(self._odom_yaw)) * odom_dt
        self._odom_y += (noisy_vx * math.sin(self._odom_yaw)) * odom_dt
        self._odom_yaw += noisy_wz * odom_dt

        qx, qy, qz, qw = quaternion_from_euler_zyx(0.0, 0.0, self._odom_yaw)

        msg = Odometry()
        msg.header.stamp = self._now_stamp()
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_link

        msg.pose.pose.position.x = self._odom_x
        msg.pose.pose.position.y = self._odom_y
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Diagonal pose covariance (xy and yaw)
        msg.pose.covariance[0] = (self._odom_lin_sigma * t) ** 2 + 0.01
        msg.pose.covariance[7] = (self._odom_lin_sigma * t) ** 2 + 0.01
        msg.pose.covariance[35] = (self._odom_ang_sigma * t) ** 2 + 0.001

        # Twist in body frame
        msg.twist.twist.linear.x = vx + \
            self._rng.normal(0, self._odom_lin_sigma)
        msg.twist.twist.linear.y = vy + \
            self._rng.normal(0, self._odom_lin_sigma)
        msg.twist.twist.angular.z = wz + \
            self._rng.normal(0, self._odom_ang_sigma)
        msg.twist.covariance[0] = self._odom_lin_sigma ** 2
        msg.twist.covariance[7] = self._odom_lin_sigma ** 2
        msg.twist.covariance[35] = self._odom_ang_sigma ** 2

        self._pub_odom.publish(msg)
        self._publish_gt(t, x, y, yaw)

    # ------------------------------------------------------------------
    # GNSS callback
    # ------------------------------------------------------------------

    def _gnss_cb(self):
        if not self._maybe_start():
            return

        t = self._elapsed()
        x, y, z, yaw, pitch, roll = self._gt_at(t)

        # Convert ENU to geodetic (flat-Earth)
        lat, lon, alt = enu_to_latlon(
            x + self._rng.normal(0, self._gnss_xy_sigma),
            y + self._rng.normal(0, self._gnss_xy_sigma),
            z + self._rng.normal(0, self._gnss_z_sigma),
        )

        msg = NavSatFix()
        msg.header.stamp = self._now_stamp()
        msg.header.frame_id = self._gnss_frame
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        # ENU covariance (xx, yy, zz on diagonal)
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        msg.position_covariance[0] = self._gnss_xy_sigma ** 2
        msg.position_covariance[4] = self._gnss_xy_sigma ** 2
        msg.position_covariance[8] = self._gnss_z_sigma ** 2

        self._pub_gnss.publish(msg)

    # ------------------------------------------------------------------
    # IMU callback
    # ------------------------------------------------------------------

    def _imu_cb(self):
        if not self._maybe_start():
            return

        t = self._elapsed()
        x, y, z, yaw, pitch, roll = self._gt_at(t)
        vx, vy, vz, wx, wy, wz = self._gt_twist_at(t)

        # Orientation quaternion (IMU convention: quat_yaw = enu_yaw - pi/2)
        noisy_yaw = yaw + self._rng.normal(0, self._imu_yaw_sigma)
        qx, qy, qz, qw = imu_quat_from_enu_pose(noisy_yaw, pitch, roll)

        # Linear acceleration in body frame
        if self._scenario == 'moving':
            ax_w, ay_w = moving_gt_world_accel(t)
        else:
            ax_w, ay_w = 0.0, 0.0
        acc = imu_linear_acc_body(yaw, pitch, roll, ax_w, ay_w)

        msg = Imu()
        msg.header.stamp = self._now_stamp()
        msg.header.frame_id = self._imu_frame

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        cov_quat = self._imu_yaw_sigma ** 2
        msg.orientation_covariance[0] = cov_quat
        msg.orientation_covariance[4] = cov_quat
        msg.orientation_covariance[8] = cov_quat

        msg.angular_velocity.x = self._rng.normal(0, math.radians(0.5))
        msg.angular_velocity.y = self._rng.normal(0, math.radians(0.5))
        msg.angular_velocity.z = wz + self._rng.normal(0, math.radians(1.0))
        msg.angular_velocity_covariance[0] = math.radians(1.0) ** 2
        msg.angular_velocity_covariance[4] = math.radians(1.0) ** 2
        msg.angular_velocity_covariance[8] = math.radians(1.0) ** 2

        msg.linear_acceleration.x = acc[0] + \
            self._rng.normal(0, self._imu_acc_sigma)
        msg.linear_acceleration.y = acc[1] + \
            self._rng.normal(0, self._imu_acc_sigma)
        msg.linear_acceleration.z = acc[2] + \
            self._rng.normal(0, self._imu_acc_sigma)
        cov_acc = self._imu_acc_sigma ** 2
        msg.linear_acceleration_covariance[0] = cov_acc
        msg.linear_acceleration_covariance[4] = cov_acc
        msg.linear_acceleration_covariance[8] = cov_acc

        self._pub_imu.publish(msg)

    # ------------------------------------------------------------------
    # Ground-truth pose publisher (called from odom callback)
    # ------------------------------------------------------------------

    def _publish_gt(self, t, x, y, yaw):
        qx, qy, qz, qw = quaternion_from_euler_zyx(0.0, 0.0, yaw)
        msg = PoseStamped()
        msg.header.stamp = self._now_stamp()
        msg.header.frame_id = 'enu'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._pub_gt.publish(msg)


def main():
    rclpy.init()
    node = SensorMockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
