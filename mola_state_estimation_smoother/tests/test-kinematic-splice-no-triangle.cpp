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
 * @file   test-kinematic-splice-no-triangle.cpp
 * @brief  Regression test: splicing a keyframe between two linked ones must
 *         withdraw their direct kinematic link, not leave it double-counted.
 * @author Jose Luis Blanco Claraco
 * @date   Jul 17, 2026
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <iostream>
#include <set>
#include <string>
#include <utility>

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

std::string navStateParams(const char* kinematicModel)
{
    return std::string(
               R"###(# Config for Parameters
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    max_time_to_use_velocity_model: 2.0
    kinematic_model: )###") +
           kinematicModel + R"###(
    sliding_window_length: 60.0
    min_time_difference_to_create_new_frame: 0.05
    sigma_random_walk_acceleration_linear: 1.0
    sigma_random_walk_acceleration_angular: 1.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.10
    sigma_twist_from_consecutive_poses_linear: 1.0
    sigma_twist_from_consecutive_poses_angular: 1.0
    estimate_geo_reference: false
)###";
}

mrpt::poses::CPose3DPDFGaussian pose_at(double x)
{
    auto cov = mrpt::math::CMatrixDouble66::Identity();
    cov *= 1e-4;
    return {mrpt::poses::CPose3D::FromXYZYawPitchRoll(x, 0, 0, 0, 0, 0), cov};
}

// Each kinematic link emits exactly two FactorConstLocalVelocityPose (one for
// linear, one for angular velocity), so the smoother's count is 2x the links.
size_t count_const_vel_factors(
    const mola::state_estimation_smoother::StateEstimationSmoother& stateEst)
{
    return stateEst.count_const_vel_factors_for_testing();
}

using link_set_t = std::set<std::pair<mrpt::Clock::time_point, mrpt::Clock::time_point>>;

void assert_link_present(const link_set_t& links, double from, double to)
{
    const bool present =
        links.count({mrpt::Clock::fromDouble(from), mrpt::Clock::fromDouble(to)}) != 0;
    std::cout << "[no-triangle] link " << from << "->" << to << " present=" << present
              << " (expected: true)\n";
    ASSERT_(present);
}

void assert_link_absent(const link_set_t& links, double from, double to)
{
    const bool present =
        links.count({mrpt::Clock::fromDouble(from), mrpt::Clock::fromDouble(to)}) != 0;
    std::cout << "[no-triangle] link " << from << "->" << to << " present=" << present
              << " (expected: false)\n";
    ASSERT_(!present);
}

// A keyframe spliced between two already-linked keyframes must not leave the
// original direct link in the graph: `a->b` plus the `a->n` and `n->b` that
// replace it would count the constant-velocity model twice over that span,
// over-stiffening the window and making the covariance over-confident.
void test_splice_does_not_leave_a_triangle(const char* kinematicModel)
{
    std::cout << "[no-triangle] kinematic_model=" << kinematicModel << "\n";

    mola::state_estimation_smoother::StateEstimationSmoother stateEst;
    if (VERBOSE)
    {
        stateEst.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }

    auto cfgYaml = mrpt::containers::yaml::FromText(navStateParams(kinematicModel));
    stateEst.initialize(cfgYaml);

    const auto& mapFrame = stateEst.parameters().reference_frame_name;

    // In-order stream at 0.2 s spacing: a plain chain of N keyframes has N-1
    // links. Spacing stays well above min_time_difference_to_create_new_frame so
    // that every pose really does create its own keyframe.
    const int kNumInOrder = 6;  // t = 0.0 .. 1.0
    for (int i = 0; i < kNumInOrder; i++)
    {
        const double t = 0.2 * i;
        stateEst.fuse_pose(mrpt::Clock::fromDouble(t), pose_at(t), mapFrame);
    }
    // Force the pending factors into iSAM2.
    (void)stateEst.estimated_navstate(mrpt::Clock::fromDouble(1.0), mapFrame);

    const size_t nBefore = count_const_vel_factors(stateEst);
    std::cout << "[no-triangle] const-vel factors for a " << kNumInOrder
              << "-keyframe chain: " << nBefore << " (expected " << 2 * (kNumInOrder - 1) << ")\n";
    ASSERT_EQUAL_(nBefore, static_cast<size_t>(2 * (kNumInOrder - 1)));

    const auto linksBefore = stateEst.const_vel_factor_links_for_testing();
    assert_link_present(linksBefore, 0.4, 0.6);

    // Now splice: t=0.5 lands between the keyframes at 0.4 and 0.6, 0.1 s from
    // either, so it creates a keyframe in the past rather than snapping.
    stateEst.fuse_pose(mrpt::Clock::fromDouble(0.5), pose_at(0.5), mapFrame);
    (void)stateEst.estimated_navstate(mrpt::Clock::fromDouble(1.0), mapFrame);

    const size_t nAfter = count_const_vel_factors(stateEst);

    // The chain gained one keyframe, so it must gain exactly one link: the
    // 0.4->0.6 link is replaced by 0.4->0.5 and 0.5->0.6. Leaving the old link in
    // would give one link (2 factors) more than this.
    const size_t nExpected = 2 * kNumInOrder;
    std::cout << "[no-triangle] after splicing one keyframe: " << nAfter << " (expected "
              << nExpected << "; a surviving stale link would give " << nExpected + 2 << ")\n";
    ASSERT_EQUAL_(nAfter, nExpected);

    // Same check at the graph-topology level, not just the factor count: the
    // 0.4->0.6 link must be gone, replaced by 0.4->0.5 and 0.5->0.6, so a
    // count-only check can't be fooled by e.g. a stale link plus a missing new one.
    const auto linksAfter = stateEst.const_vel_factor_links_for_testing();
    assert_link_absent(linksAfter, 0.4, 0.6);
    assert_link_present(linksAfter, 0.4, 0.5);
    assert_link_present(linksAfter, 0.5, 0.6);
}
}  // namespace

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;
    try
    {
        test_splice_does_not_leave_a_triangle("KinematicModel::ConstantVelocity");
        test_splice_does_not_leave_a_triangle("KinematicModel::Tricycle");
        std::cout << "Test passed." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed:\n" << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
