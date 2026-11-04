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
 * @file   test-odometry-pose-source-fidelity.cpp
 * @brief  With one accurate odometry-frame pose source and nothing else, the
 *         fused trajectory must reproduce that source.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 4, 2026
 *
 * A walking-speed path with repeated 90 deg turns, fed through fuse_pose() in
 * an odometry frame (the way a LiDAR odometry front end does), with the
 * estimator's default kinematic model. The constant-velocity factors are a
 * prior on accelerations: if that prior is tighter than the centripetal
 * acceleration of an ordinary turn, the optimizer cuts every turn short and
 * the heading error accumulates, since the source only constrains increments.
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/Lie/SO.h>

#include <cmath>
#include <iostream>
#include <optional>

using namespace std::string_literals;
using namespace mrpt::literals;

namespace
{
const size_t NUM_POSES = 600;  // 60 s at 10 Hz
const double T         = 0.1;  // [s]
const double SPEED     = 1.3;  // [m/s]

// Straight for 10 s, then a 90 deg left turn over 3 s: 0.52 rad/s, i.e. a
// centripetal acceleration of 0.68 m/s^2.
const double STRAIGHT_TIME = 10.0;
const double TURN_TIME     = 3.0;

const double MAX_POSITION_ERROR = 0.05;  // [m]
const double MAX_ANGLE_ERROR    = 0.5;  // [deg]

const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

const char* navStateParams =
    R"###(# Config for Parameters
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 6.0
    max_time_to_use_velocity_model: 0.75
    relative_pose_increment_sigma_lin: 0.02
    relative_pose_increment_sigma_ang: 0.005
    estimate_geo_reference: false
)###";

void run_test()
{
    mola::state_estimation_smoother::StateEstimationSmoother stateEst;
    if (VERBOSE)
    {
        stateEst.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    stateEst.initialize(mrpt::containers::yaml::FromText(navStateParams));

    // The odometry frame starts somewhere arbitrary:
    mrpt::poses::CPose3D odomPose(4.0, -2.0, 0.5, 20.0_deg, 0, 0);

    std::optional<mrpt::poses::CPose3D> firstFused;
    std::optional<mrpt::poses::CPose3D> firstOdom;

    double maxPosErr  = 0;
    double maxAngErr  = 0;
    size_t numChecked = 0;

    for (size_t i = 0; i < NUM_POSES; i++)
    {
        const double t     = T * static_cast<double>(i);
        const auto   stamp = mrpt::Clock::fromDouble(1000.0 + t);

        if (i > 0)
        {
            const double phase = std::fmod(t, STRAIGHT_TIME + TURN_TIME);
            const double w     = phase > STRAIGHT_TIME ? (M_PI / 2) / TURN_TIME : 0.0;
            odomPose           = odomPose + mrpt::poses::CPose3D(SPEED * T, 0, 0, w * T, 0, 0);
        }

        mrpt::poses::CPose3DPDFGaussian pdf;
        pdf.mean = odomPose;
        pdf.cov.setDiagonal(mrpt::square(0.01));
        for (int k = 3; k < 6; k++)
        {
            pdf.cov(k, k) = mrpt::square(0.001);
        }
        stateEst.fuse_pose(stamp, pdf, "odom");

        const auto ns = stateEst.estimated_navstate(stamp, "map");
        if (!ns.has_value())
        {
            continue;
        }
        const auto fused = mrpt::poses::CPose3D(ns->pose.mean);
        if (!firstFused)
        {
            firstFused = fused;
            firstOdom  = odomPose;
            continue;
        }

        // Both trajectories relative to their first pose, so the comparison
        // does not depend on how {map} and {odom} were related:
        const auto relFused = fused - *firstFused;
        const auto relOdom  = odomPose - *firstOdom;
        const auto err      = relFused - relOdom;

        maxPosErr = std::max(maxPosErr, err.norm());
        maxAngErr = std::max(
            maxAngErr, mrpt::RAD2DEG(mrpt::poses::Lie::SO<3>::log(err.getRotationMatrix()).norm()));
        numChecked++;
    }

    std::cout << "checked " << numChecked << " poses: max position error " << maxPosErr
              << " m, max angle error " << maxAngErr << " deg\n";

    ASSERT_GT_(numChecked, NUM_POSES * 9 / 10);
    ASSERT_LT_(maxPosErr, MAX_POSITION_ERROR);
    ASSERT_LT_(maxAngErr, MAX_ANGLE_ERROR);
}
}  // namespace

int main()
{
    try
    {
        run_test();
        std::cout << "Test successful." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed:\n" << e.what() << std::endl;
        return 1;
    }
}
