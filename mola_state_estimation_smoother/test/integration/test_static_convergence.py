# Integration test — Scenario A: static vehicle, fixed geo-reference.
#
# Launches mola-cli (StateEstimationSmoother + BridgeROS2) and a Python
# sensor mock that publishes noisy odometry, GNSS, and IMU for a robot
# standing still 30 m East / 50 m North of the ENU origin at yaw=60 deg.
#
# Asserts that the estimator's published pose converges to within 0.5 m and
# 5 deg of ground truth within 60 s of wall-clock streaming.
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
import launch_testing.markers
import pytest
import rclpy
from ament_index_python import get_package_share_directory

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import PoseLatest, wrap_pi, wait_for_convergence  # noqa: E402, isort:skip

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
    static_params = os.path.join(
        pkg_share, 'test', 'integration', 'data', 'static_test_smoother_params.yaml')

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
            'MOLA_STATE_ESTIMATOR_YAML': static_params,
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
            'MOLA_VERBOSITY_BRIDGE_ROS2': 'INFO',
            'MOLA_VERBOSITY_MOLA_STATE_ESTIMATOR': 'INFO',
            'RCUTILS_LOGGING_BUFFERED_STREAM': '1',
        },
    )

    # Use ExecuteProcess to launch the mock node:
    mock_proc = launch.actions.ExecuteProcess(
        cmd=['python3', mock_script,
             '--ros-args',
             '-p', 'scenario:=static',
             '-p', 'seed:=42',
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
# Active test (runs while the launch is alive)
# ------------------------------------------------------------------

class TestStaticConvergence(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.checker = rclpy.create_node('checker_static')
        cls.est_pose = PoseLatest()
        cls.gt_pose = PoseLatest()

        def _est_cb(msg):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            cls.est_pose.update(x, y, math.atan2(siny_cosp, cosy_cosp))

        def _gt_cb(msg):
            x = msg.pose.position.x
            y = msg.pose.position.y
            q = msg.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            cls.gt_pose.update(x, y, math.atan2(siny_cosp, cosy_cosp))

        cls.checker.create_subscription(
            Odometry, 'state_estimation/pose', _est_cb, 10)
        cls.checker.create_subscription(
            PoseStamped, '/ground_truth/pose', _gt_cb, 10)

    @classmethod
    def tearDownClass(cls):
        cls.checker.destroy_node()
        rclpy.shutdown()

    def test_pose_convergence(self):
        if os.environ.get(_SKIP_ENV):
            self.skipTest('MOLA_SKIP_INTEGRATION_TESTS is set')

        wait_for_convergence(
            self.checker,
            self.gt_pose,
            self.est_pose,
            max_pos_err_m=0.5,
            max_heading_err_deg=5.0,
            settle_seconds=2.0,
            timeout_seconds=60.0,
            warm_up_seconds=5.0,
        )
