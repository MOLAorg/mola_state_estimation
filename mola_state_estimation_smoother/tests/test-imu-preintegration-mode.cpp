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
 * @file   test-imu-preintegration-mode.cpp
 * @brief  imu_preintegration.mode parsing, including the deprecated boolean
 *         `enabled` flag and boolean mode values.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 5, 2026
 */

#include <mola_state_estimation_smoother/Parameters.h>
#include <mrpt/core/exceptions.h>

#include <iostream>
#include <string>

namespace
{
using mola::state_estimation_smoother::ImuPreintegrationMode;
using mola::state_estimation_smoother::Parameters;

ImuPreintegrationMode load(const std::string& block)
{
    const auto cfg = mrpt::containers::yaml::FromText(
        "vehicle_frame_name: base_link\n"
        "reference_frame_name: map\n"
        "kinematic_model: KinematicModel::ConstantVelocity\n"
        "max_time_to_use_velocity_model: 2.0\n"
        "sliding_window_length: 3.0\n"
        "relative_pose_increment_sigma_lin: 0.02\n"
        "relative_pose_increment_sigma_ang: 0.005\n" +
        block);
    Parameters p;
    p.loadFrom(cfg);
    return p.imu_preintegration.mode;
}
}  // namespace

int main()
{
    try
    {
        ASSERT_(load("") == Parameters().imu_preintegration.mode);
        ASSERT_(load("imu_preintegration:\n  mode: none\n") == ImuPreintegrationMode::None);
        ASSERT_(load("imu_preintegration:\n  mode: gyro\n") == ImuPreintegrationMode::Gyro);
        ASSERT_(load("imu_preintegration:\n  mode: Full\n") == ImuPreintegrationMode::Full);
        // Deprecated forms:
        ASSERT_(load("imu_preintegration:\n  enabled: true\n") == ImuPreintegrationMode::Full);
        ASSERT_(load("imu_preintegration:\n  enabled: false\n") == ImuPreintegrationMode::None);
        ASSERT_(load("imu_preintegration:\n  mode: true\n") == ImuPreintegrationMode::Full);
        ASSERT_(load("imu_preintegration:\n  mode: false\n") == ImuPreintegrationMode::None);

        bool threw = false;
        try
        {
            (void)load("imu_preintegration:\n  mode: both\n");
        }
        catch (const std::exception&)
        {
            threw = true;
        }
        ASSERT_(threw);

        std::cout << "Test successful." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed:\n" << e.what() << std::endl;
        return 1;
    }
}
