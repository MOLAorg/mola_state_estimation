/* _
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
 * @file   test-odometry-relative-same-keyframe.cpp
 * @brief  Verifies the relative wheel-odometry chain keeps the motion of
 *         readings that land on an already existing keyframe.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose2D.h>

#include <cmath>
#include <iostream>
#include <string>

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

// Faster than the keyframe grid (min_time_difference_to_create_new_frame is
// 10 ms), so every other reading lands on the keyframe of the previous one.
constexpr double ODOM_DT      = 0.005;  // [s]  (200 Hz)
constexpr double DURATION     = 2.0;  // [s]
constexpr double VELOCITY_X   = 1.0;  // [m/s]
constexpr size_t NUM_READINGS = static_cast<size_t>(DURATION / ODOM_DT) + 1;

const char* PARAMS_YAML = R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    link_first_pose_to_reference_origin_sigma: 1e-6
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 10.0
    max_time_to_use_velocity_model: 2.0
    sigma_random_walk_acceleration_linear: 2.0
    sigma_random_walk_acceleration_angular: 1.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.10
    estimate_geo_reference: false
    min_time_difference_to_create_new_frame: 0.01
    odometry_min_sample_period: 0.0
)###";

void run_test()
{
    mola::state_estimation_smoother::StateEstimationSmoother est;
    if (VERBOSE)
    {
        est.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    est.initialize(mrpt::containers::yaml::FromText(PARAMS_YAML));

    mrpt::Clock::time_point lastStamp;
    for (size_t i = 0; i < NUM_READINGS; i++)
    {
        const double t     = ODOM_DT * static_cast<double>(i);
        const auto   stamp = mrpt::Clock::fromDouble(t);
        lastStamp          = stamp;

        mrpt::obs::CObservationOdometry odom;
        odom.timestamp = stamp;
        odom.odometry  = mrpt::poses::CPose2D(VELOCITY_X * t, 0.0, 0.0);

        est.fuse_odometry(odom, "wheels_odom");
    }

    const auto navOpt = est.estimated_navstate(lastStamp, "map");
    ASSERT_(navOpt.has_value());
    const double finalX = navOpt->pose.mean.x();

    std::cout << "final x: " << finalX << " (expected " << DURATION * VELOCITY_X << ")\n";

    // Dropping the increments of same-keyframe readings would leave the
    // estimate short by about half the traveled distance.
    ASSERT_(std::isfinite(finalX));
    ASSERT_LT_(std::abs(finalX - DURATION * VELOCITY_X), 0.10);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        run_test();
        std::cout << "✅ SUCCESS\n";
        return 0;
    }
    catch (std::exception& e)
    {
        std::cerr << "❌ FAILED: " << e.what() << std::endl;
        return 1;
    }
}
