# ROS 2 launch file: StateEstimationSmoother demo with simulated sensors.
#
# Runs the synthetic sensor publisher from mola_demos (robot driving a circle
# of 5 m radius), the smoother fed with the sensor combination chosen by
# `mode`, and RViz showing the fused pose (with covariance), the raw
# odometries, and the ground truth.
#
# Usage:
#   ros2 launch mola_state_estimation_smoother ros2-demo-simulated-sensors.launch.py \
#       mode:=wheels_imu_gnss
#
# Modes (all of them publish the fused map -> base_link pose):
#   wheels_imu       : wheel odometry + IMU.
#   wheels_imu_gnss  : wheel odometry + IMU + GNSS, estimating the geo-reference.
#   two_odometries   : wheel odometry + drifting visual odometry + IMU.
#   imu_gnss         : IMU + GNSS only (no odometry), estimating the geo-reference.
#
# The simulated odometries and the ground truth start at the origin of {enu}
# with yaw=0, and so does {map} here; the static identity transforms published
# below rely on this to draw everything in the same RViz fixed frame.

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os

# mode -> (odom1, odom2, gnss, estimate_geo_reference)
MODES = {
    'wheels_imu': ('/wheel_odom', '', '', False),
    'wheels_imu_gnss': ('/wheel_odom', '', '/gps', True),
    'two_odometries': ('/wheel_odom', '/visual_odom', '', False),
    'imu_gnss': ('', '', '/gps', True),
}


def _launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in MODES:
        raise RuntimeError(
            f"Unknown mode '{mode}'. Valid modes: {', '.join(MODES)}")
    odom1, odom2, gnss, estimate_georef = MODES[mode]

    myDir = get_package_share_directory('mola_state_estimation_smoother')
    fake_sensors_script = os.path.join(
        get_package_share_directory('mola_demos'), 'demos',
        'fake_sensor_publisher.py')

    # The simulator always publishes every sensor: `mode` only selects what
    # the smoother subscribes to.
    fake_sensors = ExecuteProcess(
        cmd=['python3', fake_sensors_script, '--ros-args',
             '-p', 'scenario:=circle',
             '-p', 'odom_topic:=/wheel_odom',
             '-p', 'odom2_topic:=/visual_odom',
             '-p', 'imu_topic:=/imu',
             '-p', 'gnss_topic:=/gps',
             # Noisier than the simulator defaults, so odometry drift is
             # visible within one lap:
             '-p', 'odom_ang_sigma:=' + LaunchConfiguration('wheel_odom_ang_sigma').perform(context),
             '-p', 'odom2_drift_y_per_step:=' + LaunchConfiguration('visual_odom_drift').perform(context)],
        output='screen')

    smoother = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(myDir, 'ros2-launchs', 'ros2-state-estimator.launch.py')),
        launch_arguments={
            'odom1_topic': odom1,
            'odom1_label': 'wheel_odom',
            'odom2_topic': odom2,
            'odom2_label': 'visual_odom',
            'imu_topic_name': '/imu',
            'gnss_topic_name': gnss,
            'estimate_geo_reference': str(estimate_georef),
            'use_mola_gui': LaunchConfiguration('use_mola_gui'),
        }.items())

    def static_tf(parent, child):
        return Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'static_tf_{parent}_to_{child}',
            arguments=['--frame-id', parent, '--child-frame-id', child],
            output='log')

    # Raw odometries drawn from their own (fixed) starting point, so their
    # drift with respect to the fused estimate is visible:
    actions = [fake_sensors, smoother,
               static_tf('map', 'odom'), static_tf('map', 'odom_visual')]

    # With an estimated geo-reference, {enu} -> {map} is published by the
    # smoother once it converges. Otherwise, both frames coincide here:
    if not estimate_georef:
        actions.append(static_tf('enu', 'map'))

    actions.append(Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(
            myDir, 'rviz2', 'state_estimation_demo.rviz')],
        output='log'))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='wheels_imu',
            description='Sensors fused by the smoother: ' + ', '.join(MODES)),
        DeclareLaunchArgument(
            'wheel_odom_ang_sigma', default_value='0.15',
            description='Simulated wheel odometry yaw rate noise [rad/s]'),
        DeclareLaunchArgument(
            'visual_odom_drift', default_value='0.08',
            description='Simulated visual odometry lateral drift [m/s]'),
        DeclareLaunchArgument(
            'use_rviz', default_value='True', description='Launch RViz2'),
        DeclareLaunchArgument(
            'use_mola_gui', default_value='False',
            description='Also show the MolaViz GUI (console messages)'),
        OpaqueFunction(function=_launch_setup),
    ])
