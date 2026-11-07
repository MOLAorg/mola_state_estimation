/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this odometry package
 alone or in combination with the complete SLAM system.
*/

/**
 * @file   Parameters.cpp
 * @brief  Parameters for NavStateFuse
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */

#include <mola_state_estimation_smoother/Parameters.h>

namespace mola::state_estimation_smoother
{

void Parameters::loadFrom(const mrpt::containers::yaml& cfg)
{
    // Reference frame IDs
    // -----------------------------------------------------
    MCP_LOAD_REQ(cfg, vehicle_frame_name);
    MCP_LOAD_REQ(cfg, reference_frame_name);
    MCP_LOAD_OPT(cfg, enu_frame_name);
    MCP_LOAD_OPT(cfg, publish_map_to_odom_tf);
    MCP_LOAD_OPT(cfg, map_to_odom_frame_name);
    MCP_LOAD_OPT(cfg, map_to_odom_child_frame);
    MCP_LOAD_OPT(cfg, map_to_odom_extra_frames);
    MCP_LOAD_OPT(cfg, publish_fused_vehicle_tf);
    MCP_LOAD_OPT(cfg, fused_vehicle_frame_name);
    MCP_LOAD_OPT(cfg, publish_imu_yaw_bias);
    MCP_LOAD_OPT(cfg, publish_imu_biases);
    MCP_LOAD_OPT(cfg, publish_georef_convergence);

    // Kinematic factors and keyframe creation (motion model)
    // -------------------------------------------------------
    MCP_LOAD_REQ(cfg, max_time_to_use_velocity_model);

    MCP_LOAD_REQ(cfg, sliding_window_length);
    MCP_LOAD_OPT(cfg, keep_finalized_trajectory);

    MCP_LOAD_OPT(cfg, sigma_random_walk_acceleration_linear);
    MCP_LOAD_OPT(cfg, sigma_random_walk_acceleration_angular);
    MCP_LOAD_OPT(cfg, predict_twist_filter_enabled);
    MCP_LOAD_OPT(cfg, predict_twist_filter_time_const);
    MCP_LOAD_OPT(cfg, predict_covariance_from_residuals);
    MCP_LOAD_OPT(cfg, predict_residual_time_const);
    // Wheel odometry is always fused as relative increments. The key is still
    // accepted when set to true, so existing configuration files keep working.
    if (cfg.has("odometry_relative_factors"))
    {
        ASSERTMSG_(
            cfg["odometry_relative_factors"].as<bool>(),
            "odometry_relative_factors=false is no longer supported: wheel odometry is always "
            "fused as relative increments between keyframes");
    }
    MCP_LOAD_OPT(cfg, sigma_integrator_position);
    MCP_LOAD_OPT(cfg, sigma_integrator_orientation);
    MCP_LOAD_OPT(cfg, sigma_relative_pose_linear);
    MCP_LOAD_OPT(cfg, sigma_relative_pose_angular);
    MCP_LOAD_OPT(cfg, sigma_twist_from_consecutive_poses_linear);
    MCP_LOAD_OPT(cfg, sigma_twist_from_consecutive_poses_angular);

    MCP_LOAD_OPT(cfg, min_time_difference_to_create_new_frame);
    MCP_LOAD_OPT(cfg, time_between_frames_to_warning);
    MCP_LOAD_OPT(cfg, gnss_nearby_keyframe_stamp_tolerance);
    MCP_LOAD_OPT(cfg, imu_nearby_keyframe_stamp_tolerance);

    MCP_LOAD_OPT(cfg, odometry_min_sample_period);
    MCP_LOAD_OPT(cfg, imu_min_sample_period);

    MCP_LOAD_OPT(cfg, odom_motion_model_a1);
    MCP_LOAD_OPT(cfg, odom_motion_model_a2);
    MCP_LOAD_OPT(cfg, odom_motion_model_a3);
    MCP_LOAD_OPT(cfg, odom_motion_model_a4);
    MCP_LOAD_OPT(cfg, odom_motion_model_min_std_xy);
    MCP_LOAD_OPT(cfg, odom_motion_model_min_std_phi_deg);
    MCP_LOAD_OPT(cfg, odom_out_of_plane_std_z);
    MCP_LOAD_OPT(cfg, odom_out_of_plane_std_roll_pitch);

    MCP_LOAD_OPT(cfg, async_backend);
    MCP_LOAD_OPT(cfg, fast_predictor_buffer_length);

    MCP_LOAD_OPT(cfg, initial_twist_sigma_lin);
    MCP_LOAD_OPT(cfg, initial_twist_sigma_ang);

    MCP_LOAD_OPT(cfg, enforce_planar_motion);

    if (cfg.has("link_first_pose_to_reference_origin_sigma"))
    {
        const auto strSigma = cfg["link_first_pose_to_reference_origin_sigma"].as<std::string>();
        double     sigma    = 0;
        if (1 == ::sscanf(strSigma.c_str(), "%lf", &sigma))
        {
            ASSERT_GT_(sigma, .0);
            link_first_pose_to_reference_origin_sigma = sigma;
        }
    }

    // IMU-related
    // -----------------------------------------------------
    MCP_LOAD_OPT(cfg, imu_attitude_sigma_deg);
    MCP_LOAD_OPT(cfg, imu_attitude_azimuth_offset_deg);
    MCP_LOAD_OPT(cfg, imu_attitude_yaw_bias_sigma_deg);
    MCP_LOAD_OPT(cfg, imu_attitude_yaw_bias_time_constant);
    MCP_LOAD_OPT(cfg, imu_attitude_min_sample_period);
    ASSERT_GE_(imu_attitude_yaw_bias_sigma_deg, 0.0);
    ASSERT_GT_(imu_attitude_yaw_bias_time_constant, 0.0);
    MCP_LOAD_OPT(cfg, imu_normalized_gravity_alignment_sigma);
    MCP_LOAD_OPT(cfg, imu_gravity_min_sample_period);
    MCP_LOAD_OPT(cfg, imu_gravity_compensate_centripetal);
    MCP_LOAD_OPT(cfg, imu_angular_velocity_sigma);
    MCP_LOAD_OPT(cfg, imu_bias_tracking);
    MCP_LOAD_OPT(cfg, imu_bias_labels_re);
    MCP_LOAD_OPT(cfg, imu_bias_correct_angular_velocity);
    MCP_LOAD_OPT(cfg, imu_bias_hint_max_speed);
    MCP_LOAD_OPT(cfg, imu_bias_hint_max_angular_speed_deg);
    MCP_LOAD_OPT(cfg, imu_bias_hint_min_period);
    if (cfg.has("imu_bias_tracker"))
    {
        const mrpt::containers::yaml sub = cfg["imu_bias_tracker"];
        imu_bias_tracker                 = sub;
    }
    if (cfg.has("imu_preintegration"))
    {
        const mrpt::containers::yaml c  = cfg["imu_preintegration"];
        auto&                        ip = imu_preintegration;
        ip.enabled                      = c.getOrDefault("enabled", ip.enabled);
        ip.sensor_label_re              = c.getOrDefault("sensor_label_re", ip.sensor_label_re);
        ip.gyro_noise_density  = c.getOrDefault("gyro_noise_density", ip.gyro_noise_density);
        ip.accel_noise_density = c.getOrDefault("accel_noise_density", ip.accel_noise_density);
        ip.gyro_bias_random_walk =
            c.getOrDefault("gyro_bias_random_walk", ip.gyro_bias_random_walk);
        ip.accel_bias_random_walk =
            c.getOrDefault("accel_bias_random_walk", ip.accel_bias_random_walk);
        ip.integration_sigma = c.getOrDefault("integration_sigma", ip.integration_sigma);
        ip.gravity           = c.getOrDefault("gravity", ip.gravity);
        ip.initial_gyro_bias_sigma =
            c.getOrDefault("initial_gyro_bias_sigma", ip.initial_gyro_bias_sigma);
        ip.initial_accel_bias_sigma =
            c.getOrDefault("initial_accel_bias_sigma", ip.initial_accel_bias_sigma);
        ip.linear_kinematics_sigma_scale =
            c.getOrDefault("linear_kinematics_sigma_scale", ip.linear_kinematics_sigma_scale);
        ip.max_reading_gap    = c.getOrDefault("max_reading_gap", ip.max_reading_gap);
        ip.min_sigma_rotation = c.getOrDefault("min_sigma_rotation", ip.min_sigma_rotation);
        ip.min_sigma_position = c.getOrDefault("min_sigma_position", ip.min_sigma_position);
        ip.min_sigma_velocity = c.getOrDefault("min_sigma_velocity", ip.min_sigma_velocity);
        ip.min_link_duration  = c.getOrDefault("min_link_duration", ip.min_link_duration);
        ASSERT_GT_(ip.gyro_noise_density, 0.0);
        ASSERT_GT_(ip.accel_noise_density, 0.0);
        ASSERT_GT_(ip.gyro_bias_random_walk, 0.0);
        ASSERT_GT_(ip.accel_bias_random_walk, 0.0);
        ASSERT_GE_(ip.linear_kinematics_sigma_scale, 1.0);
    }

    // Geo-referencing
    // -----------------------------------------------------
    MCP_LOAD_OPT(cfg, gnss_huber_threshold);
    MCP_LOAD_OPT(cfg, gnss_robust_kernel);
    ASSERTMSG_(
        gnss_robust_kernel == "huber" || gnss_robust_kernel == "cauchy" ||
            gnss_robust_kernel == "geman_mcclure" || gnss_robust_kernel == "dcs",
        "gnss_robust_kernel must be one of: huber, cauchy, geman_mcclure, dcs");
    MCP_LOAD_OPT(cfg, gnss_accepted_fix_qualities);
    ASSERTMSG_(
        gnss_accepted_fix_qualities.find_first_not_of("0123456789 ,\t") == std::string::npos,
        "gnss_accepted_fix_qualities must be a list of integers, e.g. \"4 5\"");
    MCP_LOAD_OPT(cfg, gnss_max_horizontal_sigma);
    MCP_LOAD_OPT(cfg, gnss_max_height_deviation);
    MCP_LOAD_OPT(cfg, gnss_gate_chi2);
    MCP_LOAD_OPT(cfg, gnss_gate_prediction_sigma);
    MCP_LOAD_OPT(cfg, gnss_gate_drift_per_meter);
    MCP_LOAD_OPT(cfg, gnss_gate_reacquire_after);
    MCP_LOAD_OPT(cfg, gnss_gate_reacquire_min_path);
    ASSERT_GE_(gnss_gate_reacquire_min_path, 0.0);
    MCP_LOAD_OPT(cfg, gnss_max_plausible_speed);
    MCP_LOAD_OPT(cfg, gnss_datum_min_consistent_fixes);
    ASSERT_GE_(gnss_max_horizontal_sigma, 0.0);
    ASSERT_GE_(gnss_max_height_deviation, 0.0);
    ASSERT_GE_(gnss_gate_chi2, 0.0);
    ASSERT_GT_(gnss_gate_prediction_sigma, 0.0);
    ASSERT_GE_(gnss_gate_drift_per_meter, 0.0);
    ASSERT_GE_(gnss_gate_reacquire_after, 0);
    ASSERT_GT_(gnss_max_plausible_speed, 0.0);
    ASSERT_GE_(gnss_datum_min_consistent_fixes, 1);
    MCP_LOAD_OPT(cfg, gnss_min_sigma_floor_xy);
    MCP_LOAD_OPT(cfg, gnss_min_sigma_floor_z);
    ASSERT_GE_(gnss_min_sigma_floor_xy, 0.0);
    ASSERT_GE_(gnss_min_sigma_floor_z, 0.0);
    MCP_LOAD_OPT(cfg, save_final_georef_to_file);
    MCP_LOAD_OPT(cfg, estimate_geo_reference);

    if (cfg.has("fixed_geo_reference"))
    {
        auto& gr = fixed_geo_reference.emplace();

        const auto& fgr = cfg["fixed_geo_reference"];

        ASSERT_(fgr.isMap());
        ASSERT_(fgr.has("latitude_deg"));
        ASSERT_(fgr.has("longitude_deg"));
        ASSERT_(fgr.has("altitude"));

        gr.geo_coord.lat    = fgr["latitude_deg"].as<double>();
        gr.geo_coord.lon    = fgr["longitude_deg"].as<double>();
        gr.geo_coord.height = fgr["altitude"].as<double>();

        // TODO: Allow defining a custom transformation != Identity?
        gr.T_enu_to_map.cov.setIdentity();
        gr.T_enu_to_map.cov *= 1e-6;
    }

    MCP_LOAD_OPT(cfg, convergence_max_position_sigma);
    MCP_LOAD_OPT(cfg, convergence_max_orientation_sigma_deg);
    MCP_LOAD_OPT(cfg, publish_estimated_georef_on_convergence);

    MCP_LOAD_REQ(cfg, kinematic_model);

    // Nonlinear optimization
    // -----------------------------------------------------
    MCP_LOAD_OPT(cfg, additional_isam2_update_steps);
    MCP_LOAD_OPT(cfg, isam2_relinearize_threshold);

    // name Sensor input names
    // -----------------------------------------------------
    MCP_LOAD_OPT(cfg, do_process_imu_labels_re);
    MCP_LOAD_OPT(cfg, do_process_odometry_labels_re);
    MCP_LOAD_OPT(cfg, fuse_ground_truth_label);
    MCP_LOAD_OPT(cfg, pose_min_sample_period);
    MCP_LOAD_REQ(cfg, relative_pose_increment_sigma_lin);
    MCP_LOAD_REQ(cfg, relative_pose_increment_sigma_ang);
    MCP_LOAD_OPT(cfg, relative_pose_increment_sigma_per_sqrt_meter);
    MCP_LOAD_OPT(cfg, relative_pose_increment_sigma_per_sqrt_rad);
    MCP_LOAD_OPT(cfg, pose_robust_huber_threshold);
    MCP_LOAD_OPT(cfg, relative_pose_increment_from_source_covariance);
    MCP_LOAD_OPT(cfg, do_process_gnss_labels_re);

    ASSERTMSG_(pose_min_sample_period >= 0, "pose_min_sample_period must be >= 0");
    ASSERTMSG_(
        relative_pose_increment_sigma_lin > 0 && relative_pose_increment_sigma_ang > 0,
        "relative_pose_increment_sigma_lin/_ang must be > 0: they define the uncertainty of "
        "each increment of an odometry source fused with fuse_pose()");
    ASSERTMSG_(
        relative_pose_increment_sigma_per_sqrt_meter >= 0 &&
            relative_pose_increment_sigma_per_sqrt_rad >= 0,
        "relative_pose_increment_sigma_per_sqrt_meter/_rad must be >= 0");
    ASSERTMSG_(pose_robust_huber_threshold >= 0, "pose_robust_huber_threshold must be >= 0");

    if (cfg.has("initial_twist"))
    {
        ASSERT_(cfg["initial_twist"].isSequence() && cfg["initial_twist"].asSequence().size() == 6);

        auto&      tw  = initial_twist;
        const auto seq = cfg["initial_twist"].asSequenceRange();
        for (size_t i = 0; i < 6; i++)
        {
            tw[i] = seq.at(i).as<double>();
        }
    }

    if (cfg.has("visualization"))
    {
        visualization.loadFrom(cfg["visualization"]);
    }
}

void Parameters::Visualization::loadFrom(const mrpt::containers::yaml& cfg)
{
    MCP_LOAD_OPT(cfg, show_console_messages);
}

}  // namespace mola::state_estimation_smoother
