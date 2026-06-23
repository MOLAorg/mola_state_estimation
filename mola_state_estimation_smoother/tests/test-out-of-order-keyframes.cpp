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
 * @file   test-out-of-order-keyframes.cpp
 * @brief  Unit tests for the out-of-order keyframe guard: late (out-of-timestamp
 *         order) measurements must snap to the nearest existing keyframe instead
 *         of inserting a new one "in the past", so iSAM2's fixed-lag
 *         marginalization never corrupts.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 23, 2026
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/poses/CPose3D.h>

#include <iostream>
#include <memory>

using namespace std::string_literals;

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

std::unique_ptr<mola::state_estimation_smoother::StateEstimationSmoother> makeEstimator(
    double slidingWindow)
{
    const std::string params =
        R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    link_first_pose_to_reference_origin_sigma: 1e-6
    kinematic_model: KinematicModel::ConstantVelocity
    min_time_difference_to_create_new_frame: 0.01
    max_time_to_use_velocity_model: 2.0
    sigma_random_walk_acceleration_linear: 2.0
    sigma_random_walk_acceleration_angular: 1.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.10
    estimate_geo_reference: false
    sliding_window_length: )###" +
        std::to_string(slidingWindow) + "\n";

    auto est = std::make_unique<mola::state_estimation_smoother::StateEstimationSmoother>();
    if (VERBOSE)
    {
        est->setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    est->initialize(mrpt::containers::yaml::FromText(params));
    return est;
}

// Absolute "map"-frame pose observation at time t along a straight line (x = t).
mrpt::poses::CPose3DPDFGaussian poseAt(double t)
{
    mrpt::poses::CPose3DPDFGaussian p;
    p.mean = mrpt::poses::CPose3D(t * 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    p.cov.setIdentity();
    p.cov *= 1e-3;
    return p;
}

// --------------------------------------------------------------------------
// Test 1: an out-of-order pose snaps to an existing keyframe instead of creating
//         a new one in the past (observable via the keyframe count).
// --------------------------------------------------------------------------
void test_out_of_order_pose_snaps_no_new_keyframe()
{
    auto est = makeEstimator(30.0);

    // Build an in-order chain of keyframes at t = 0.0 .. 1.0 (0.1 spacing).
    for (int i = 0; i <= 10; i++)
    {
        est->fuse_pose(mrpt::Clock::fromDouble(0.1 * i), poseAt(0.1 * i), "map");
    }
    const std::size_t kfBefore = est->active_keyframe_count();
    std::cout << "[snap] keyframes after in-order chain=" << kfBefore << "\n";
    ASSERT_GT_(kfBefore, static_cast<std::size_t>(5));

    // A late pose for t=0.55 arrives now (out of order: 0.45 s behind the newest
    // keyframe at t=1.0, and >min_time_difference from any existing one). The
    // guard must snap it to the nearest existing keyframe, NOT create a new one.
    est->fuse_pose(mrpt::Clock::fromDouble(0.55), poseAt(0.55), "map");

    const std::size_t kfAfter = est->active_keyframe_count();
    std::cout << "[snap] keyframes after out-of-order pose=" << kfAfter << "\n";
    ASSERT_EQUAL_(kfAfter, kfBefore);  // no new (past) keyframe was inserted
}

// --------------------------------------------------------------------------
// Test 2: a stream with persistent out-of-order arrivals crossing the
//         marginalization horizon does not throw (this is the exact failure mode
//         that crashed iSAM2 before the guard).
// --------------------------------------------------------------------------
void test_out_of_order_with_marginalization_does_not_throw()
{
    auto est = makeEstimator(2.0);  // short window -> marginalization is exercised

    // Advance well past the window so old keyframes are marginalized out, while
    // continuously injecting "late" poses ~0.3 s in the past (mimics a lagged
    // LiDAR front-end fused on top of high-rate keyframes).
    for (int i = 0; i <= 200; i++)
    {
        const double t = 0.05 * i;  // 20 Hz "high-rate" stream, t = 0 .. 10 s
        est->fuse_pose(mrpt::Clock::fromDouble(t), poseAt(t), "map");

        // Every few steps, inject a late pose 0.3 s behind:
        if (i >= 10 && (i % 3) == 0)
        {
            const double tLate = t - 0.3;
            est->fuse_pose(mrpt::Clock::fromDouble(tLate), poseAt(tLate), "map");
        }

        // Force smoother updates (and thus marginalization) along the way:
        if ((i % 5) == 0)
        {
            const auto st = est->estimated_navstate(mrpt::Clock::fromDouble(t), "map");
            ASSERT_(st.has_value());
        }
    }

    // Final query must still succeed and track the straight line (x = t).
    const double last_t = 0.05 * 200;
    const auto   st     = est->estimated_navstate(mrpt::Clock::fromDouble(last_t), "map");
    ASSERT_(st.has_value());
    std::cout << "[marg] final est x=" << st->pose.mean.x() << " (expected ~" << last_t << ")\n";
    ASSERT_NEAR_(st->pose.mean.x(), last_t, 0.2);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_out_of_order_pose_snaps_no_new_keyframe();
        test_out_of_order_with_marginalization_does_not_throw();

        std::cout << "✅ SUCCESS\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "❌ FAILED: " << e.what() << std::endl;
        return 1;
    }
}
