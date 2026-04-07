# ROS 2 launch file: Fuse two external odometry sources via the MOLA smoother
#
# This is a convenience wrapper around the central state_estimator_ros2.yaml,
# pre-configuring it for two nav_msgs/Odometry inputs + optional IMU/GNSS.
#
# Usage:
#   ros2 launch mola_state_estimation_smoother ros2-fuse-two-odometries.launch.py \
#       odom1_topic:=/wheel_odom odom2_topic:=/visual_odom
#
# The fused output is published as:
#   - nav_msgs/Odometry on topic: state_estimation/pose
#   - /tf: map -> base_link

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable,
                            GroupAction, Shutdown)
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    myDir = get_package_share_directory("mola_state_estimation_smoother")

    # -------------------
    #     Arguments
    # -------------------
    odom1_topic_arg = DeclareLaunchArgument(
        "odom1_topic", default_value="/wheel_odom",
        description="First nav_msgs/Odometry topic (e.g. wheel encoders)")
    odom1_env = SetEnvironmentVariable(
        name='ODOM1_TOPIC', value=LaunchConfiguration('odom1_topic'))

    odom1_label_arg = DeclareLaunchArgument(
        "odom1_label", default_value="wheel_odom",
        description="Sensor label for first odometry source")
    odom1_label_env = SetEnvironmentVariable(
        name='ODOM1_LABEL', value=LaunchConfiguration('odom1_label'))

    odom2_topic_arg = DeclareLaunchArgument(
        "odom2_topic", default_value="/visual_odom",
        description="Second nav_msgs/Odometry topic (e.g. visual/lidar odometry)")
    odom2_env = SetEnvironmentVariable(
        name='ODOM2_TOPIC', value=LaunchConfiguration('odom2_topic'))

    odom2_label_arg = DeclareLaunchArgument(
        "odom2_label", default_value="visual_odom",
        description="Sensor label for second odometry source")
    odom2_label_env = SetEnvironmentVariable(
        name='ODOM2_LABEL', value=LaunchConfiguration('odom2_label'))

    imu_topic_arg = DeclareLaunchArgument(
        "imu_topic", default_value="/imu",
        description="IMU topic for gravity alignment (empty to disable)")
    imu_env = SetEnvironmentVariable(
        name='IMU_TOPIC', value=LaunchConfiguration('imu_topic'))

    gnss_topic_arg = DeclareLaunchArgument(
        "gnss_topic", default_value="",
        description="GNSS topic for geo-referencing (empty to disable)")
    gnss_env = SetEnvironmentVariable(
        name='GNSS_TOPIC', value=LaunchConfiguration('gnss_topic'))

    use_mola_gui_arg = DeclareLaunchArgument(
        "use_mola_gui", default_value="True",
        description="Show MolaViz GUI")
    use_mola_gui_env = SetEnvironmentVariable(
        name='MOLA_WITH_GUI', value=LaunchConfiguration('use_mola_gui'))

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="False",
        description="Launch RViz2")

    mola_tf_base_link_arg = DeclareLaunchArgument(
        "mola_tf_base_link", default_value="base_link")
    mola_tf_base_link_env = SetEnvironmentVariable(
        name='MOLA_TF_BASE_LINK', value=LaunchConfiguration('mola_tf_base_link'))

    enforce_planar_motion_arg = DeclareLaunchArgument(
        "enforce_planar_motion", default_value="False",
        description="Enforce z=0, pitch=0, roll=0")
    enforce_planar_motion_env = SetEnvironmentVariable(
        name='MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION',
        value=LaunchConfiguration('enforce_planar_motion'))

    # Anchor first pose to map origin (useful for local demos without geo-ref):
    link_first_pose_env = SetEnvironmentVariable(
        name='MOLA_LINK_FIRST_POSE_SIGMA', value='1e-6')

    # Publish fused estimates from the smoother:
    publish_tf_source_env = SetEnvironmentVariable(
        name='MOLA_LOCALIZATION_PUBLISH_TF_SOURCE', value='state_estimation')
    publish_odom_source_env = SetEnvironmentVariable(
        name='MOLA_LOCALIZATION_PUBLISH_ODOM_MSGS_SOURCE', value='state_estimation')

    # Namespace
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace', default_value='false',
        description='Whether to apply a namespace')

    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    mola_system_yaml_file = os.path.join(
        myDir, 'mola-cli-launchs', 'state_estimator_ros2.yaml')

    node_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),
        Node(
            package='mola_launcher',
            executable='mola-cli',
            output='screen',
            remappings=tf_remaps,
            arguments=[mola_system_yaml_file],
            on_exit=Shutdown()
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            remappings=tf_remaps,
        )
    ])

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        odom1_topic_arg, odom1_env,
        odom1_label_arg, odom1_label_env,
        odom2_topic_arg, odom2_env,
        odom2_label_arg, odom2_label_env,
        imu_topic_arg, imu_env,
        gnss_topic_arg, gnss_env,
        use_mola_gui_arg, use_mola_gui_env,
        use_rviz_arg,
        mola_tf_base_link_arg, mola_tf_base_link_env,
        enforce_planar_motion_arg, enforce_planar_motion_env,
        link_first_pose_env,
        publish_tf_source_env,
        publish_odom_source_env,
        node_group
    ])
