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
 * @file   test-pose-keyframe-decimation.cpp
 * @brief  Verifies pose_min_sample_period thins a high-rate fuse_pose() source
 *         without losing motion, under both the absolute and the relative
 *         formulation.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <cmath>
#include <iostream>
#include <string>
#include <utility>

using namespace std::string_literals;

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

// A pose source an order of magnitude faster than the keyframe rate anyone
// would want, which is the situation this parameter exists for.
constexpr double POSE_DT      = 0.005;  // [s]  (200 Hz)
constexpr double DURATION     = 2.0;  // [s]
constexpr double VELOCITY_X   = 1.0;  // [m/s]
constexpr double DECIM_PERIOD = 0.10;  // [s]  (target ~10 Hz)
constexpr size_t NUM_READINGS = static_cast<size_t>(DURATION / POSE_DT) + 1;  // 401
constexpr double SOURCE_SIGMA = 0.02;  // [m]

std::string params_yaml(double poseMinSamplePeriod, bool relative)
{
    return
        R"###(
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
    relative_factors_frame_ids_re: ")###" +
        (relative ? ".*odom.*"s : ""s) + R"###("
    pose_min_sample_period: )###" +
        std::to_string(poseMinSamplePeriod) + "\n";
}

/// Feeds a constant-velocity pose source and returns {live kinematic links,
/// final estimated map-frame x}.
std::pair<size_t, double> run(double poseMinSamplePeriod, bool relative)
{
    mola::state_estimation_smoother::StateEstimationSmoother est;
    if (VERBOSE)
    {
        est.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    est.initialize(mrpt::containers::yaml::FromText(params_yaml(poseMinSamplePeriod, relative)));

    mrpt::Clock::time_point lastStamp;
    for (size_t i = 0; i < NUM_READINGS; i++)
    {
        const double t     = POSE_DT * static_cast<double>(i);
        const auto   stamp = mrpt::Clock::fromDouble(t);
        lastStamp          = stamp;

        mrpt::poses::CPose3DPDFGaussian pdf;
        pdf.mean =
            mrpt::poses::CPose3D::FromXYZYawPitchRoll(VELOCITY_X * t, 0.0, 0.0, 0.0, 0.0, 0.0);
        for (int k = 0; k < 3; k++)
        {
            pdf.cov(k, k) = SOURCE_SIGMA * SOURCE_SIGMA;
        }
        for (int k = 3; k < 6; k++)
        {
            pdf.cov(k, k) = mrpt::square(0.01);
        }

        est.fuse_pose(stamp, pdf, "legged_odom");
    }

    // Query first: this flushes pending factors into the smoother, so the
    // topology below is the final one.
    const auto   navOpt = est.estimated_navstate(lastStamp, "map");
    const double finalX = navOpt.has_value() ? navOpt->pose.mean.x() : std::nan("");
    const auto   links  = est.const_vel_factor_links_for_testing();

    return {links.size(), finalX};
}

void check(bool relative)
{
    const char* mode = relative ? "relative" : "absolute";

    const auto [linksFull, finalXFull]   = run(0.0, relative);
    const auto [linksDecim, finalXDecim] = run(DECIM_PERIOD, relative);

    std::cout << "[" << mode << "] links: " << linksFull << " -> " << linksDecim
              << ", final x: " << finalXFull << " -> " << finalXDecim << "\n";

    // The vehicle must still arrive at 2.0 m. A dropped reading that also lost
    // its motion would leave the estimate short by whole merged segments, so
    // this is the property that matters, not the keyframe count.
    ASSERT_(std::isfinite(finalXFull));
    ASSERT_(std::isfinite(finalXDecim));
    ASSERT_LT_(std::abs(finalXFull - DURATION * VELOCITY_X), 0.10);
    ASSERT_LT_(std::abs(finalXDecim - DURATION * VELOCITY_X), 0.15);
    ASSERT_LT_(std::abs(finalXFull - finalXDecim), 0.15);

    // And the thinning must be real, several-fold, not a no-op.
    ASSERT_GT_(linksFull, static_cast<size_t>(100));
    ASSERT_LT_(linksDecim, static_cast<size_t>(40));
    ASSERT_LT_(linksDecim * 3, linksFull);
}

void run_test()
{
    check(false);
    check(true);
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
