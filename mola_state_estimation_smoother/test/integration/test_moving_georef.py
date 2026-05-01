# Integration test — Scenario B: moving robot, self-estimated geo-reference.
#
# Launches mola-cli with estimate_geo_reference=true and a sensor mock that
# publishes a sinusoidal 60-second trajectory (v=1 m/s, A=2 m, omega=0.5 rad/s).
#
# Asserts that after a 20-second warm-up the estimator's pose tracks the
# ground truth within 1.5 m and 10 deg.
import math
import os
import sys
import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import PoseLatest, wait_for_convergence  # noqa: E402, isort:skip

_SKIP_ENV = 'MOLA_SKIP_INTEGRATION_TESTS'

# ------------------------------------------------------------------
# Launch description
# ------------------------------------------------------------------


@pytest.mark.launch_test
def generate_test_description():
    if os.environ.get(_SKIP_ENV):
        return launch.LaunchDescription([
            launch_testing.actions.ReadyToTest(),
        ])

    pkg_share = get_package_share_directory('mola_state_estimation_smoother')
    mola_yaml = os.path.join(
        pkg_share, 'mola-cli-launchs', 'state_estimator_ros2.yaml')
    moving_params = os.path.join(
        pkg_share, 'test', 'integration', 'data', 'moving_test_smoother_params.yaml')

    mock_script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               'sensor_mock_node.py')

    mola_node = launch_ros.actions.Node(
        package='mola_launcher',
        executable='mola-cli',
        output='screen',
        arguments=[mola_yaml],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        additional_env={
            'MOLA_WITH_GUI': 'false',
            'MOLA_STATE_ESTIMATOR_YAML': moving_params,
            'ODOM1_TOPIC': '/wheel_odom',
            'ODOM1_LABEL': 'wheel_odom',
            'IMU_TOPIC': '/imu',
            'GNSS_TOPIC': '/gps',
            'ODOM2_TOPIC': '',  # remove after all distros have mola_yaml>=2.6.1
            'ODOM3_TOPIC': '',  # remove after all distros have mola_yaml>=2.6.1
            'ODOM4_TOPIC': '',  # remove after all distros have mola_yaml>=2.6.1
            'MOLA_USE_FIXED_IMU_POSE': 'true',
            'IMU_POSE_X': '0', 'IMU_POSE_Y': '0', 'IMU_POSE_Z': '0',
            'IMU_POSE_YAW': '0', 'IMU_POSE_PITCH': '0', 'IMU_POSE_ROLL': '0',
            'MOLA_USE_FIXED_GNSS_POSE': 'true',
            'GNSS_POSE_X': '0', 'GNSS_POSE_Y': '0', 'GNSS_POSE_Z': '0',
            'GNSS_POSE_YAW': '0', 'GNSS_POSE_PITCH': '0', 'GNSS_POSE_ROLL': '0',
            'MOLA_LOCALIZATION_PUBLISH_ODOM_MSGS': 'true',
            'MOLA_LOCALIZATION_PUBLISH_ODOM_MSGS_SOURCE': 'state_estimation',
            'MOLA_LOCALIZATION_PUBLISH_TF': 'true',
            'MOLA_LOCALIZATION_PUBLISH_TF_SOURCE': 'state_estimation',
            'MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION': 'true',
            'MOLA_ESTIMATE_GEO_REF': 'true',
            'MOLA_VERBOSITY_BRIDGE_ROS2': 'INFO',
            'MOLA_VERBOSITY_MOLA_STATE_ESTIMATOR': 'INFO',
            'RCUTILS_LOGGING_BUFFERED_STREAM': '1',
        },
    )

    mock_proc = launch.actions.ExecuteProcess(
        cmd=['python3', mock_script,
             '--ros-args',
             '-p', 'scenario:=moving',
             '-p', 'seed:=1234',
             '-p', 'startup_delay_sec:=4.0',
             '-p', 'odom_rate:=10.0',
             '-p', 'gnss_rate:=2.0',
             '-p', 'imu_rate:=20.0'],
        output='screen',
    )

    return launch.LaunchDescription([
        mola_node,
        mock_proc,
        launch_testing.actions.ReadyToTest(),
    ])


# ------------------------------------------------------------------
# Active test
# ------------------------------------------------------------------

class TestMovingGeoRef(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.checker = rclpy.create_node('checker_moving')
        cls.est_pose = PoseLatest()
        cls.gt_pose = PoseLatest()

        def _yaw_from_quat(q):
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny_cosp, cosy_cosp)

        def _est_cb(msg):
            cls.est_pose.update(
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                _yaw_from_quat(msg.pose.pose.orientation))

        def _gt_cb(msg):
            cls.gt_pose.update(
                msg.pose.position.x,
                msg.pose.position.y,
                _yaw_from_quat(msg.pose.orientation))

        cls.checker.create_subscription(
            Odometry, 'state_estimation/pose', _est_cb, 10)
        cls.checker.create_subscription(
            PoseStamped, '/ground_truth/pose', _gt_cb, 10)

    @classmethod
    def tearDownClass(cls):
        cls.checker.destroy_node()
        rclpy.shutdown()

    def test_moving_trajectory_tracking(self):
        if os.environ.get(_SKIP_ENV):
            self.skipTest('MOLA_SKIP_INTEGRATION_TESTS is set')

        # The estimator publishes pose in the `map` frame, while ground truth is
        # in the `enu` frame. With link_first_pose_to_reference_origin_sigma=1e-6
        # the smoother anchors the robot's first pose to the map origin with
        # identity orientation, so map == enu only if the robot starts at ENU
        # origin with yaw=0. In this scenario GT(t=0) = (0, 0, atan2(A·omega, V))
        # = (0, 0, 45°), so T_enu_to_map is a -45° rotation about the origin and
        # the two frames are NOT directly comparable.
        #
        # We recover T_map_from_enu from the first matched (gt, est) pair
        # captured during the warm-up window, then transform GT into map.
        max_pos_err_m = 1.5
        max_heading_err_deg = 10.0
        settle_seconds = 3.0
        warm_up_seconds = 20.0
        timeout_seconds = 40.0

        deadline = time.monotonic() + warm_up_seconds + timeout_seconds
        warm_up_end = time.monotonic() + warm_up_seconds
        settled_since = None
        pos_err = float('inf')
        yaw_err_deg = float('inf')
        # (tx, ty, dyaw) such that map = R(dyaw)·enu + (tx,ty)
        T_map_from_enu = None

        while time.monotonic() < deadline:
            rclpy.spin_once(self.checker, timeout_sec=0.05)

            now = time.monotonic()

            gt, gt_t = self.gt_pose.latest()
            est, est_t = self.est_pose.latest()
            fresh = (gt is not None and est is not None and
                     gt_t is not None and est_t is not None and
                     now - gt_t < 0.5 and now - est_t < 0.5)

            # Lock T_map_from_enu using the very first fresh sample. At t≈0 the
            # estimator's pose equals the map-anchored origin, so this pair pins
            # the transform between the two frames for the rest of the run.
            if T_map_from_enu is None and fresh:
                dyaw = est[2] - gt[2]
                c, s = math.cos(dyaw), math.sin(dyaw)
                tx = est[0] - (c * gt[0] - s * gt[1])
                ty = est[1] - (s * gt[0] + c * gt[1])
                T_map_from_enu = (tx, ty, dyaw)
                print(f'[moving] locked T_map_from_enu=({tx:.3f},{ty:.3f},'
                      f'{math.degrees(dyaw):.2f}°)', flush=True)

            if now < warm_up_end:
                continue

            if not fresh or T_map_from_enu is None:
                continue

            tx, ty, dyaw = T_map_from_enu
            c, s = math.cos(dyaw), math.sin(dyaw)
            gt_in_map_x = c * gt[0] - s * gt[1] + tx
            gt_in_map_y = s * gt[0] + c * gt[1] + ty
            gt_in_map_yaw = gt[2] + dyaw

            pos_err = math.hypot(gt_in_map_x - est[0], gt_in_map_y - est[1])
            yaw_err = math.degrees(
                math.atan2(math.sin(gt_in_map_yaw - est[2]),
                           math.cos(gt_in_map_yaw - est[2])))
            yaw_err_deg = abs(yaw_err)
            passed = pos_err < max_pos_err_m and yaw_err_deg < max_heading_err_deg

            print(
                f'[moving] gt_map=({gt_in_map_x:.2f},{gt_in_map_y:.2f},'
                f'{math.degrees(gt_in_map_yaw):.1f}°) '
                f'est_map=({est[0]:.2f},{est[1]:.2f},{math.degrees(est[2]):.1f}°) '
                f'pos_err={pos_err:.3f}m yaw_err={yaw_err:.1f}° pass={passed}',
                flush=True)

            if passed:
                if settled_since is None:
                    settled_since = now
                if now - settled_since >= settle_seconds:
                    print(f'[moving] CONVERGED after {settle_seconds:.1f}s settled',
                          flush=True)
                    return
            else:
                settled_since = None

        raise AssertionError(
            f'did not converge within {timeout_seconds:.0f}s: '
            f'pos_err={pos_err:.2f}m yaw_err={yaw_err_deg:.2f}deg'
        )
