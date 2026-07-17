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
 * @file   test-navstate-stamp-monotonic.cpp
 * @brief  Regression test: a late measurement must not drag the published
 *         timestamp backwards.
 * @author Jose Luis Blanco Claraco
 * @date   Jul 17, 2026
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <iostream>
#include <optional>

using namespace std::string_literals;

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

const char* navStateParams =
    R"###(# Config for Parameters
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    max_time_to_use_velocity_model: 2.0
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 5.0
    min_time_difference_to_create_new_frame: 0.05
    sigma_random_walk_acceleration_linear: 1.0
    sigma_random_walk_acceleration_angular: 1.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.10
    sigma_twist_from_consecutive_poses_linear: 1.0
    sigma_twist_from_consecutive_poses_angular: 1.0
    estimate_geo_reference: false
)###";

mrpt::poses::CPose3DPDFGaussian pose_at(double x)
{
    auto cov = mrpt::math::CMatrixDouble66::Identity();
    cov *= 1e-4;
    return {mrpt::poses::CPose3D::FromXYZYawPitchRoll(x, 0, 0, 0, 0, 0), cov};
}

// The smoother extrapolates the published stamp in real time as
//   last_observation_stamp + (wallclock now - wallclock when it was received).
// Feeding measurements in order and then a LATE one must not move that reference
// back to the late measurement's stamp: doing so makes every subsequently
// published pose claim a timestamp in the past, and then creep forward again as
// wallclock advances, i.e. a sawtooth on the stamp of the module's output.
void test_late_measurement_does_not_regress_published_stamp()
{
    mola::state_estimation_smoother::StateEstimationSmoother stateEst;
    if (VERBOSE)
    {
        stateEst.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }

    auto cfgYaml = mrpt::containers::yaml::FromText(navStateParams);
    stateEst.initialize(cfgYaml);

    // spinOnce() only publishes when someone is listening:
    std::optional<mrpt::Clock::time_point> lastPublishedStamp;
    stateEst.subscribeToLocalizationUpdates(
        [&lastPublishedStamp](const mola::LocalizationSourceBase::LocalizationUpdate& lu)
        { lastPublishedStamp = lu.timestamp; });

    const auto& mapFrame = stateEst.parameters().reference_frame_name;

    // A short, in-order stream. The newest keyframe ends up at t=1.0.
    //
    // Keyframe spacing (0.2 s) is deliberately kept well above
    // min_time_difference_to_create_new_frame (0.05 s), so that the late
    // measurement below lands in a GAP and really does create a keyframe in the
    // past. With a denser grid it would instead snap to an existing keyframe and
    // return early, never reaching the code under test -- which is exactly how
    // this regression hides in the field.
    const double kNewestInOrder = 1.0;
    const double kKeyframeStep  = 0.2;
    for (double t = 0.0; t <= kNewestInOrder + 1e-9; t += kKeyframeStep)
    {
        stateEst.fuse_pose(mrpt::Clock::fromDouble(t), pose_at(t), mapFrame);
    }

    stateEst.spinOnce();
    ASSERT_(lastPublishedStamp.has_value());
    const double stampBefore = mrpt::Clock::toDouble(*lastPublishedStamp);

    // Now a LATE measurement arrives, stamped well before the newest keyframe.
    // This is the normal case for a CPU-bound front end feeding fuse_pose()
    // directly while higher-rate sources have already advanced the graph.
    // t=0.5 sits midway between the keyframes at 0.4 and 0.6, i.e. 0.1 s from
    // either, comfortably outside the 0.05 s snap threshold.
    const double kLateStamp = 0.5;
    stateEst.fuse_pose(mrpt::Clock::fromDouble(kLateStamp), pose_at(kLateStamp), mapFrame);

    stateEst.spinOnce();
    ASSERT_(lastPublishedStamp.has_value());
    const double stampAfter = mrpt::Clock::toDouble(*lastPublishedStamp);

    std::cout << "[stamp-monotonic] published stamp before late measurement: " << stampBefore
              << "\n[stamp-monotonic] published stamp after  late measurement: " << stampAfter
              << "\n[stamp-monotonic] late measurement stamp: " << kLateStamp << "\n";

    // The published stamp must not go backwards...
    ASSERT_GE_(stampAfter, stampBefore);

    // ...and specifically it must still track the newest keyframe (1.0), not the
    // late one (0.5). Both spinOnce() calls happen within milliseconds of each
    // other, so the real-time extrapolation added to the reference is tiny; a
    // regression would show up here as ~0.5 instead of ~1.0.
    const double kTolerance = 0.1;  // generous: only wallclock jitter lives in here
    ASSERT_GT_(stampAfter, kLateStamp + kTolerance);
    ASSERT_NEAR_(stampAfter, kNewestInOrder, kTolerance);
}
}  // namespace

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;
    try
    {
        test_late_measurement_does_not_regress_published_stamp();
        std::cout << "Test passed." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed:\n" << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
