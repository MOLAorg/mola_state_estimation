# ROS 2 launch file for the MOLA State Estimation Smoother.
#
# This is the general-purpose launch file that exposes the full set of
# smoother and BridgeROS2 parameters as launch arguments.  It calls the
# central state_estimator_ros2.yaml.
#
# Usage:
#   ros2 launch mola_state_estimation_smoother ros2-state-estimator.launch.py \
#       imu_topic_name:=/imu gnss_topic_name:=/gps

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable,
                            UnsetEnvironmentVariable, GroupAction, Shutdown,
                            OpaqueFunction)
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():

    # ~~~~~~~~~~~~~~~~~~~~~~~~
    #  Smoother arguments
    # ~~~~~~~~~~~~~~~~~~~~~~~~
    navstate_kinematic_model_arg = DeclareLaunchArgument(
        "navstate_kinematic_model",
        default_value="KinematicModel::ConstantVelocity",
        description="Kinematic model. Options: KinematicModel::ConstantVelocity, KinematicModel::Tricycle.")

    navstate_sliding_window_sec_arg = DeclareLaunchArgument(
        "navstate_sliding_window_sec",
        default_value="2.5",
        description="Time window to keep past observations [seconds].")

    navstate_sigma_random_walk_linacc_arg = DeclareLaunchArgument(
        "navstate_sigma_random_walk_linacc",
        default_value="1.0",
        description="Random walk linear acceleration uncertainty [m/s²].")

    navstate_sigma_random_walk_angacc_arg = DeclareLaunchArgument(
        "navstate_sigma_random_walk_angacc",
        default_value="10.0",
        description="Random walk angular acceleration uncertainty [rad/s²].")

    estimate_geo_reference_arg = DeclareLaunchArgument(
        "estimate_geo_reference",
        default_value="False",
        description="Estimate geo-referencing from GNSS readings.")

    odom1_topic_arg = DeclareLaunchArgument(
        "odom1_topic", default_value="",
        description="1st nav_msgs/Odometry topic (empty to disable)")
    odom1_label_arg = DeclareLaunchArgument(
        "odom1_label", default_value="odom1",
        description="Sensor label for 1st odometry source")

    odom2_topic_arg = DeclareLaunchArgument(
        "odom2_topic", default_value="",
        description="2nd nav_msgs/Odometry topic (empty to disable)")
    odom2_label_arg = DeclareLaunchArgument(
        "odom2_label", default_value="odom2",
        description="Sensor label for 2nd odometry source")

    odom3_topic_arg = DeclareLaunchArgument(
        "odom3_topic", default_value="",
        description="3rd nav_msgs/Odometry topic (empty to disable)")
    odom3_label_arg = DeclareLaunchArgument(
        "odom3_label", default_value="odom3",
        description="Sensor label for 3rd odometry source")

    smoother_env_vars = GroupAction(actions=[
        SetEnvironmentVariable('MOLA_NAVSTATE_KINEMATIC_MODEL',
                               LaunchConfiguration('navstate_kinematic_model')),
        SetEnvironmentVariable('MOLA_NAVSTATE_SLIDING_WINDOW_SEC',
                               LaunchConfiguration('navstate_sliding_window_sec')),
        SetEnvironmentVariable('MOLA_NAVSTATE_SIGMA_RANDOM_WALK_LINACC',
                               LaunchConfiguration('navstate_sigma_random_walk_linacc')),
        SetEnvironmentVariable('MOLA_NAVSTATE_SIGMA_RANDOM_WALK_ANGACC',
                               LaunchConfiguration('navstate_sigma_random_walk_angacc')),
        SetEnvironmentVariable('MOLA_ESTIMATE_GEO_REF',
                               LaunchConfiguration('estimate_geo_reference')),
        SetEnvironmentVariable('ODOM1_TOPIC', LaunchConfiguration('odom1_topic')),
        SetEnvironmentVariable('ODOM1_LABEL', LaunchConfiguration('odom1_label')),
        SetEnvironmentVariable('ODOM2_TOPIC', LaunchConfiguration('odom2_topic')),
        SetEnvironmentVariable('ODOM2_LABEL', LaunchConfiguration('odom2_label')),
        SetEnvironmentVariable('ODOM3_TOPIC', LaunchConfiguration('odom3_topic')),
        SetEnvironmentVariable('ODOM3_LABEL', LaunchConfiguration('odom3_label')),
    ])

    # ~~~~~~~~~~~~~~~~~~~~~~~~
    # BridgeROS2 arguments
    # ~~~~~~~~~~~~~~~~~~~~~~~~
    ignore_lidar_pose_from_tf_arg = DeclareLaunchArgument(
        "ignore_lidar_pose_from_tf", default_value="false",
        description="If true, LiDAR pose is assumed at base_link origin.")
    ignore_lidar_pose_from_tf_env = SetEnvironmentVariable(
        name='MOLA_USE_FIXED_LIDAR_POSE',
        value=LaunchConfiguration('ignore_lidar_pose_from_tf'))

    ignore_imu_pose_from_tf_arg = DeclareLaunchArgument(
        "ignore_imu_pose_from_tf", default_value="false",
        description="If true, IMU pose is assumed at base_link origin.")
    ignore_imu_pose_from_tf_env = SetEnvironmentVariable(
        name='MOLA_USE_FIXED_IMU_POSE',
        value=LaunchConfiguration('ignore_imu_pose_from_tf'))

    gnss_topic_name_arg = DeclareLaunchArgument(
        "gnss_topic_name", default_value="",
        description="GNSS NavSatFix topic (empty to disable)")
    gnss_topic_env = SetEnvironmentVariable(
        name='GNSS_TOPIC', value=LaunchConfiguration('gnss_topic_name'))

    imu_topic_name_arg = DeclareLaunchArgument(
        "imu_topic_name", default_value="",
        description="IMU topic (empty to disable)")
    imu_topic_env = SetEnvironmentVariable(
        name='IMU_TOPIC', value=LaunchConfiguration('imu_topic_name'))

    use_mola_gui_arg = DeclareLaunchArgument(
        "use_mola_gui", default_value="True",
        description="Show MolaViz GUI")
    use_mola_gui_env = SetEnvironmentVariable(
        name='MOLA_WITH_GUI', value=LaunchConfiguration('use_mola_gui'))

    mola_se_reference_frame_arg = DeclareLaunchArgument(
        "mola_state_estimator_reference_frame", default_value="map",
        description="Reference /tf frame for pose publication")
    mola_tf_map_env = SetEnvironmentVariable(
        name='MOLA_TF_MAP',
        value=LaunchConfiguration('mola_state_estimator_reference_frame'))

    mola_footprint_to_base_link_tf_arg = DeclareLaunchArgument(
        "mola_footprint_to_base_link_tf", default_value="[0, 0, 0, 0, 0, 0]",
        description="Transform base_footprint -> base_link")
    mola_footprint_to_base_link_tf_env = SetEnvironmentVariable(
        name='MOLA_TF_FOOTPRINT_TO_BASE_LINK',
        value=LaunchConfiguration('mola_footprint_to_base_link_tf'))

    enforce_planar_motion_arg = DeclareLaunchArgument(
        "enforce_planar_motion", default_value="False",
        description="Enforce z=0, pitch=0, roll=0")
    enforce_planar_motion_env = SetEnvironmentVariable(
        name='MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION',
        value=LaunchConfiguration('enforce_planar_motion'))

    forward_ros_tf_odom_to_mola_arg = DeclareLaunchArgument(
        "forward_ros_tf_odom_to_mola", default_value="False",
        description="Import existing /tf odom->base_link as odometry.")
    forward_ros_tf_odom_to_mola_env = SetEnvironmentVariable(
        name='MOLA_FORWARD_ROS_TF_ODOM_TO_MOLA',
        value=LaunchConfiguration('forward_ros_tf_odom_to_mola'))

    mola_tf_base_link_arg = DeclareLaunchArgument(
        "mola_tf_base_link", default_value="base_link",
        description="The /tf frame name for the robot base link")
    mola_tf_base_link_env = SetEnvironmentVariable(
        name='MOLA_TF_BASE_LINK', value=LaunchConfiguration('mola_tf_base_link'))

    localization_publish_tf_source_env = SetEnvironmentVariable(
        name='MOLA_LOCALIZATION_PUBLISH_TF_SOURCE',
        value='state_estimation')
    localization_publish_odom_source_env = SetEnvironmentVariable(
        name='MOLA_LOCALIZATION_PUBLISH_ODOM_MSGS_SOURCE',
        value='state_estimation')

    def _set_link_first_pose_sigma(context, *args, **kwargs):
        # When estimating geo-reference the map frame is defined by where the robot
        # starts, so we must anchor the first pose to the map origin (sigma=1e-6).
        # When localizing in a pre-built geo-referenced map the robot can start
        # anywhere, so MOLA_LINK_FIRST_POSE_SIGMA must be left empty.
        estimate_geo_ref = LaunchConfiguration(
            'estimate_geo_reference').perform(context).strip().lower()
        if estimate_geo_ref in ('true', '1', 'yes'):
            return [SetEnvironmentVariable(
                name='MOLA_LINK_FIRST_POSE_SIGMA', value='1e-6')]
        return [UnsetEnvironmentVariable(name='MOLA_LINK_FIRST_POSE_SIGMA')]

    link_first_pose_sigma_action = OpaqueFunction(
        function=_set_link_first_pose_sigma)

    # Namespace
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace', default_value='false',
        description='Whether to apply a namespace')

    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    myDir = get_package_share_directory("mola_state_estimation_smoother")
    mola_system_yaml_file = os.path.join(
        myDir, 'mola-cli-launchs', 'state_estimator_ros2.yaml')

    # -------------------
    #        Node
    # -------------------
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
        )
    ])

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        # Smoother
        navstate_kinematic_model_arg,
        navstate_sliding_window_sec_arg,
        navstate_sigma_random_walk_linacc_arg,
        navstate_sigma_random_walk_angacc_arg,
        estimate_geo_reference_arg,
        odom1_topic_arg, odom1_label_arg,
        odom2_topic_arg, odom2_label_arg,
        odom3_topic_arg, odom3_label_arg,
        smoother_env_vars,
        # BridgeROS2
        enforce_planar_motion_arg, enforce_planar_motion_env,
        forward_ros_tf_odom_to_mola_arg, forward_ros_tf_odom_to_mola_env,
        gnss_topic_name_arg, gnss_topic_env,
        ignore_imu_pose_from_tf_arg, ignore_imu_pose_from_tf_env,
        ignore_lidar_pose_from_tf_arg, ignore_lidar_pose_from_tf_env,
        imu_topic_name_arg, imu_topic_env,
        mola_footprint_to_base_link_tf_arg, mola_footprint_to_base_link_tf_env,
        mola_se_reference_frame_arg, mola_tf_map_env,
        mola_tf_base_link_arg, mola_tf_base_link_env,
        use_mola_gui_arg, use_mola_gui_env,
        localization_publish_tf_source_env,
        localization_publish_odom_source_env,
        link_first_pose_sigma_action,
        # Node
        node_group
    ])
