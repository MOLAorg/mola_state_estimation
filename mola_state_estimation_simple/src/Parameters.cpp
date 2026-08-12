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
 * @brief  Parameters for IMU preintegration.
 * @author Jose Luis Blanco Claraco
 * @date   Sep 19, 2021
 */

#include <mola_state_estimation_simple/Parameters.h>

#include <cmath>

namespace mola::state_estimation_simple
{

void Parameters::loadFrom(const mrpt::containers::yaml& cfg)
{
    MCP_LOAD_REQ(cfg, max_time_to_use_velocity_model);

    MCP_LOAD_OPT(cfg, sigma_random_walk_acceleration_linear);
    MCP_LOAD_OPT(cfg, sigma_random_walk_acceleration_angular);

    MCP_LOAD_OPT(cfg, sigma_relative_pose_linear);
    MCP_LOAD_OPT(cfg, sigma_relative_pose_angular);

    MCP_LOAD_OPT(cfg, sigma_imu_angular_velocity);

    MCP_LOAD_OPT(cfg, sigma_wheel_odom_linear_vel);
    MCP_LOAD_OPT(cfg, sigma_wheel_odom_angular_vel);

    MCP_LOAD_OPT(cfg, velocity_filter_enabled);

    MCP_LOAD_OPT(cfg, enforce_planar_motion);

    MCP_LOAD_OPT(cfg, do_process_imu_labels_re);
    MCP_LOAD_OPT(cfg, do_process_odometry_labels_re);
    MCP_LOAD_OPT(cfg, do_process_gnss_labels_re);

    MCP_LOAD_OPT(cfg, gnss_enabled);
    MCP_LOAD_OPT(cfg, gnss_max_horizontal_sigma);
    MCP_LOAD_OPT(cfg, gnss_min_sigma_floor_xy);
    MCP_LOAD_OPT(cfg, gnss_min_sigma_floor_z);
    MCP_LOAD_OPT(cfg, gnss_fuse_z);

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

    MCP_LOAD_OPT(cfg, initial_twist_sigma_lin);
    MCP_LOAD_OPT(cfg, initial_twist_sigma_ang);

    if (initial_twist != mrpt::math::TTwist3D())
    {
        // A zero, negative, or non-finite sigma would build a singular
        // last_twist_cov, which later throws out of inverse_LLt() in
        // estimated_navstate(). Catch it here, at config-load time, instead.
        ASSERT_(std::isfinite(initial_twist_sigma_lin) && initial_twist_sigma_lin > 0);
        ASSERT_(std::isfinite(initial_twist_sigma_ang) && initial_twist_sigma_ang > 0);
    }
}

}  // namespace mola::state_estimation_simple
