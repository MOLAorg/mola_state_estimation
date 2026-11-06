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
 * @file   test-planar-odometry-out-of-plane.cpp
 * @brief  odom_out_of_plane_std_*: what a planar wheel odometry asserts about
 *         the height, pitch and roll it does not measure.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>

#include <cmath>
#include <iostream>
#include <string>

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr double      T          = 0.1;  // period of each source [s]
constexpr double      OFFSET     = 0.05;  // between the two sources [s]
constexpr double      V          = 1.0;  // forward speed [m/s]
constexpr double      CLIMB      = 0.3;  // vertical speed, with a level body [m/s]
constexpr size_t      NUM_STEPS  = 40;
constexpr const char* POSE_FRAME = "odom";
constexpr const char* WHEELS     = "wheels";

std::string make_config(const double outOfPlaneStd)
{
    return std::string(R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    max_time_to_use_velocity_model: 2.0
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 5.0
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
    odom_out_of_plane_std_z: )###") +
           std::to_string(outOfPlaneStd) +
           "\n    odom_out_of_plane_std_roll_pitch: " + std::to_string(outOfPlaneStd) + "\n";
}

// A level body that moves up as it goes forward (e.g. a legged robot on a
// staircase): its height changes between readings in its own frame, which a
// planar wheel odometry cannot see.
mrpt::poses::CPose3D source_pose(const double t)
{
    return mrpt::poses::CPose3D::FromXYZYawPitchRoll(V * t, 0, CLIMB * t, 0, 0, 0);
}

// Height gained in {map} by the fused estimate, over the whole run:
double fused_climb(const double outOfPlaneStd)
{
    mola::state_estimation_smoother::StateEstimationSmoother est;
    {
        auto cfg = mrpt::containers::yaml::FromText(make_config(outOfPlaneStd));
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

    double tLast = 0;
    for (size_t i = 0; i < NUM_STEPS; i++)
    {
        const double tS = T * static_cast<double>(i);
        const double tW = tS + OFFSET;

        mrpt::poses::CPose3DPDFGaussian p;
        p.mean = source_pose(tS);
        p.cov.setDiagonal(mrpt::square(0.02));
        est.fuse_pose(mrpt::Clock::fromDouble(tS), p, POSE_FRAME);

        mrpt::obs::CObservationOdometry o;
        o.timestamp   = mrpt::Clock::fromDouble(tW);
        o.sensorLabel = WHEELS;
        o.odometry    = mrpt::poses::CPose2D(V * tW, 0, 0);
        est.fuse_odometry(o, WHEELS);

        [[maybe_unused]] const auto st = est.estimated_navstate(
            mrpt::Clock::fromDouble(tW), est.parameters().reference_frame_name);
        tLast = tS;
    }

    const auto st = est.estimated_navstate(
        mrpt::Clock::fromDouble(tLast), est.parameters().reference_frame_name);
    ASSERT_(st.has_value());
    return st->pose.mean.z();
}

void test_out_of_plane_std()
{
    const double trueClimb = source_pose(T * static_cast<double>(NUM_STEPS - 1)).z();
    const double planar    = fused_climb(0.0);
    const double loose     = fused_climb(1.0);

    if (VERBOSE)
    {
        std::cout << "climb: true=" << trueClimb << " planar default=" << planar
                  << " out-of-plane std 1=" << loose << "\n";
    }

    // The default asserts planar motion, and it does pull the height down:
    ASSERT_LT_(planar, 0.8 * trueClimb);
    // A loose out-of-plane uncertainty leaves the height to the pose source:
    ASSERT_LT_(std::abs(loose - trueClimb), 0.05 * trueClimb);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_out_of_plane_std();
        std::cout << "Test successful." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
