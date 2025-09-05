/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
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
    MCP_LOAD_REQ(cfg, max_time_to_use_velocity_model);

    MCP_LOAD_REQ(cfg, sliding_window_length);

    MCP_LOAD_OPT(cfg, sigma_random_walk_acceleration_linear);
    MCP_LOAD_OPT(cfg, sigma_random_walk_acceleration_angular);
    MCP_LOAD_OPT(cfg, sigma_integrator_position);
    MCP_LOAD_OPT(cfg, sigma_integrator_orientation);
    MCP_LOAD_OPT(cfg, sigma_twist_from_consecutive_poses_linear);
    MCP_LOAD_OPT(cfg, sigma_twist_from_consecutive_poses_angular);

    MCP_LOAD_OPT(cfg, time_between_frames_to_warning);

    MCP_LOAD_OPT(cfg, initial_twist_sigma_lin);
    MCP_LOAD_OPT(cfg, initial_twist_sigma_ang);

    MCP_LOAD_OPT(cfg, max_rmse);
    MCP_LOAD_OPT(cfg, robust_param);

    MCP_LOAD_OPT(cfg, enforce_planar_motion);

    MCP_LOAD_OPT(cfg, vehicle_frame_name);
    MCP_LOAD_OPT(cfg, reference_frame_name);

    {
        std::string do_process_imu_labels;
        MCP_LOAD_OPT(cfg, do_process_imu_labels);
        do_process_imu_labels_re = do_process_imu_labels;
    }

    {
        std::string do_process_odometry_labels;
        MCP_LOAD_OPT(cfg, do_process_odometry_labels);
        do_process_odometry_labels_re = do_process_odometry_labels;
    }
    {
        std::string do_process_gnss_labels;
        MCP_LOAD_OPT(cfg, do_process_gnss_labels);
        do_process_gnss_labels_re = do_process_gnss_labels;
    }
    if (cfg.has("initial_twist"))
    {
        ASSERT_(cfg["initial_twist"].isSequence() && cfg["initial_twist"].asSequence().size() == 6);

        auto&      tw  = initial_twist;
        const auto seq = cfg["initial_twist"].asSequenceRange();
        for (size_t i = 0; i < 6; i++) tw[i] = seq.at(i).as<double>();
    }
}

}  // namespace mola::state_estimation_smoother
