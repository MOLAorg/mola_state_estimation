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
            'MOLA_VERBOSITY_BRIDGE_ROS2': 'DEBUG',
            'MOLA_VERBOSITY_MOLA_STATE_ESTIMATOR': 'DEBUG',
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
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            cls.est_pose.update(x, y, _yaw_from_quat(
                msg.pose.pose.orientation))

        def _gt_cb(msg):
            x = msg.pose.position.x
            y = msg.pose.position.y
            cls.gt_pose.update(x, y, _yaw_from_quat(msg.pose.orientation))

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

        # Allow 20 s for the smoother to converge on the geo-reference, then
        # check that the estimated pose tracks GT within the moving thresholds
        # for at least 3 s continuously.
        wait_for_convergence(
            self.checker,
            self.gt_pose,
            self.est_pose,
            max_pos_err_m=1.5,
            max_heading_err_deg=10.0,
            settle_seconds=3.0,
            timeout_seconds=100.0,
            warm_up_seconds=20.0,
        )
