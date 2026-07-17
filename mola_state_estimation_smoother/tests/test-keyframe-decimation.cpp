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
 * @file   test-keyframe-decimation.cpp
 * @brief  Verifies odometry_min_sample_period merges high-rate wheel odometry
 *         into fewer keyframes without losing motion.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>

#include <cmath>
#include <iostream>
#include <string>
#include <utility>

using namespace std::string_literals;
using namespace mrpt::literals;

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

// High-rate odometry: 50 Hz for 2 s.
constexpr double ODOM_DT      = 0.02;  // [s]  (50 Hz)
constexpr double DURATION     = 2.0;  // [s]
constexpr double VELOCITY_X   = 1.0;  // [m/s]
constexpr double DECIM_PERIOD = 0.10;  // [s]  (target ~10 Hz keyframes)
constexpr size_t NUM_READINGS = static_cast<size_t>(DURATION / ODOM_DT) + 1;  // 101

std::string params_yaml(double odometryMinSamplePeriod)
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
    odometry_min_sample_period: )###" +
        std::to_string(odometryMinSamplePeriod) + "\n";
}

/// Feeds NUM_READINGS of constant-velocity wheel odometry and returns
/// {number of kinematic links live in the graph, final estimated map-frame x}.
std::pair<size_t, double> run(double odometryMinSamplePeriod)
{
    mola::state_estimation_smoother::StateEstimationSmoother est;
    if (VERBOSE)
    {
        est.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    est.initialize(mrpt::containers::yaml::FromText(params_yaml(odometryMinSamplePeriod)));

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

    // Query first: this flushes pending factors into the smoother so that
    // const_vel_factor_links_for_testing() sees the final graph topology.
    const auto   navOpt = est.estimated_navstate(lastStamp, "map");
    const double finalX = navOpt.has_value() ? navOpt->pose.mean.x() : std::nan("");
    const auto   links  = est.const_vel_factor_links_for_testing();

    return {links.size(), finalX};
}

void run_test()
{
    // 1) No decimation: one keyframe per reading (minus the first, which only
    //    seeds the anchor). Baseline for the ratio.
    const auto [linksFull, finalXFull] = run(0.0);
    // 2) Decimation at 0.10 s => ~10 Hz keyframes over the 2 s run.
    const auto [linksDecim, finalXDecim] = run(DECIM_PERIOD);

    std::cout << "links (no decimation): " << linksFull << "\n";
    std::cout << "links (decimated):     " << linksDecim << "\n";
    std::cout << "final x (no decim):    " << finalXFull << "\n";
    std::cout << "final x (decimated):   " << finalXDecim << "\n";

    // The final vehicle x must reach ~2.0 m regardless of decimation: the merge
    // must not lose motion (this is the key correctness property). The decimated
    // run is served by a coarser keyframe grid, so the single-step endpoint
    // extrapolation is looser; a lost *merge* would drop whole 0.1 m segments
    // and compound far beyond this, so this bound still guards the property.
    ASSERT_(std::isfinite(finalXFull));
    ASSERT_(std::isfinite(finalXDecim));
    ASSERT_LT_(std::abs(finalXFull - DURATION * VELOCITY_X), 0.05);
    ASSERT_LT_(std::abs(finalXDecim - DURATION * VELOCITY_X), 0.15);
    // The two configurations must agree: decimation preserves the trajectory.
    ASSERT_LT_(std::abs(finalXFull - finalXDecim), 0.15);

    // No decimation: ~100 links (one per reading beyond the first).
    ASSERT_GT_(linksFull, static_cast<size_t>(90));

    // Decimated: ~ DURATION/DECIM_PERIOD = 20 links, a several-fold reduction.
    ASSERT_LT_(linksDecim, static_cast<size_t>(30));
    ASSERT_GT_(linksDecim, static_cast<size_t>(10));

    // And it must be a real reduction vs the baseline (guards against a no-op).
    ASSERT_LT_(linksDecim * 3, linksFull);
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
