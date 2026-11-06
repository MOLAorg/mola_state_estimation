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
 * @file   test-wheel-odometry-frame-collision.cpp
 * @brief  A wheel odometry labeled like the frame of a pose source must not
 *         share that frame with it.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>

#include <algorithm>
#include <cmath>
#include <iostream>

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr double      T         = 0.1;  // period of each source [s]
constexpr double      OFFSET    = 0.05;  // between the two sources [s]
constexpr double      V         = 1.2;  // forward speed [m/s]
constexpr double      CLIMB     = 0.3;  // vertical speed [m/s]
constexpr size_t      NUM_STEPS = 40;
constexpr const char* FRAME     = "odom";

constexpr auto CONFIG = R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    max_time_to_use_velocity_model: 2.0
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 3.0
    relative_pose_increment_sigma_lin: 0.02
    relative_pose_increment_sigma_ang: 0.005
    min_time_difference_to_create_new_frame: 0.01
    odometry_min_sample_period: 0.0
    sigma_random_walk_acceleration_linear: 1.0
    sigma_random_walk_acceleration_angular: 10.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.10
    sigma_twist_from_consecutive_poses_linear: 1.0
    sigma_twist_from_consecutive_poses_angular: 1.0
    estimate_geo_reference: false
    async_backend: false
)###";

using mola::state_estimation_smoother::StateEstimationSmoother;

// A 3D pose source climbing a slope, e.g. a LiDAR odometry:
mrpt::poses::CPose3D source_pose(const double t)
{
    return mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        V * t, 0, CLIMB * t, 0, -std::atan2(CLIMB, V), 0);
}

void fuse_source(StateEstimationSmoother& est, const double t)
{
    mrpt::poses::CPose3DPDFGaussian p;
    p.mean = source_pose(t);
    p.cov.setDiagonal(mrpt::square(0.02));
    est.fuse_pose(mrpt::Clock::fromDouble(t), p, FRAME);
}

// ...and a planar wheel odometry, labeled like the frame of the former:
void fuse_wheels(StateEstimationSmoother& est, const double t)
{
    mrpt::obs::CObservationOdometry o;
    o.timestamp   = mrpt::Clock::fromDouble(t);
    o.sensorLabel = FRAME;
    o.odometry    = mrpt::poses::CPose2D(V * t, 0, 0);
    est.fuse_odometry(o, FRAME);
}

void test_frames_kept_apart(const bool wheelsFirst)
{
    StateEstimationSmoother est;
    {
        auto cfg = mrpt::containers::yaml::FromText(CONFIG);
        est.initialize(cfg);
    }
    {
        auto cov = mrpt::math::CMatrixDouble66::Identity();
        cov *= 1e-6;
        est.fuse_pose(
            mrpt::Clock::fromDouble(0),
            mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D::Identity(), cov),
            est.parameters().reference_frame_name);
    }

    // As a front end does: ask for the prediction in its frame before fusing.
    // With `wheelsFirst`, the wheel odometry takes the frame name before the
    // pose source shows up (its first reading only sets its origin).
    const double tSource0    = wheelsFirst ? OFFSET : 0.0;
    const double tWheels0    = wheelsFirst ? 0.0 : OFFSET;
    const size_t firstSource = wheelsFirst ? 3 : 0;
    double       tLast       = 0;
    for (size_t i = 0; i < NUM_STEPS; i++)
    {
        const double tS = tSource0 + T * static_cast<double>(i);
        const double tW = tWheels0 + T * static_cast<double>(i);
        if (wheelsFirst)
        {
            fuse_wheels(est, tW);
        }
        if (i > firstSource)
        {
            [[maybe_unused]] const auto pred =
                est.estimated_navstate(mrpt::Clock::fromDouble(tS), FRAME);
        }
        if (i >= firstSource)
        {
            fuse_source(est, tS);
        }
        if (!wheelsFirst)
        {
            fuse_wheels(est, tW);
        }
        tLast = std::max(tS, tW);
    }
    // The newest reading is the wheel odometry's, right before the query:
    fuse_wheels(est, tLast + 0.02);

    const double tq = tLast + OFFSET;
    const auto   st = est.estimated_navstate(mrpt::Clock::fromDouble(tq), FRAME);
    ASSERT_(st.has_value());

    const auto   expected = source_pose(tq);
    const double errZ     = std::abs(st->pose.mean.z() - expected.z());
    const double errPitch = std::abs(st->pose.mean.pitch() - expected.pitch());

    const auto frames = est.known_odometry_frame_ids();

    if (VERBOSE)
    {
        std::cout << "[wheels first=" << wheelsFirst << "] prediction in '" << FRAME
                  << "': " << st->pose.mean << " expected: " << expected << " frames:";
        for (const auto& f : frames)
        {
            std::cout << " " << f;
        }
        std::cout << "\n";
    }

    // The prediction in the pose source's frame follows that source, never the
    // planar wheel odometry:
    ASSERT_LT_(errZ, 0.05);
    ASSERT_LT_(errPitch, mrpt::DEG2RAD(1.0));

    // ...which still has a frame of its own:
    ASSERT_EQUAL_(frames.size(), 2U);
    ASSERT_(frames.count(FRAME) == 1);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_frames_kept_apart(false);
        test_frames_kept_apart(true);
        std::cout << "Test successful." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
