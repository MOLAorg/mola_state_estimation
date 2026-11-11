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
 * @file   test-reset-motion-drifted-odometry.cpp
 * @brief  reset_motion() keeps {map} when the first source back is a wheel
 *         odometry that drifted since it was first aligned with {map}.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 11, 2026
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/bits_math.h>
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

constexpr double V             = 1.0;  // forward speed [m/s]
constexpr double POSE_PERIOD   = 0.1;  // [s]
constexpr double WHEELS_PERIOD = 0.05;  // [s]
constexpr double RESET_AT      = 4.0;  // [s]
constexpr double POSE_RESUMES  = 4.5;  // [s]
constexpr double T_END         = 6.0;  // [s]
// The wheel odometry reports a turn while the vehicle goes straight:
constexpr double WHEELS_YAW_DRIFT = 0.1;  // [rad/s]

constexpr auto CONFIG = R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    max_time_to_use_velocity_model: 2.0
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 3.0
    relative_pose_increment_sigma_lin: 0.005
    relative_pose_increment_sigma_ang: 0.001
    min_time_difference_to_create_new_frame: 0.01
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

// Dead reckoning of the wheel odometry, which does not start at its own origin:
mrpt::poses::CPose2D wheels_pose(const double t)
{
    const double r = V / WHEELS_YAW_DRIFT;
    const double a = WHEELS_YAW_DRIFT * t;
    return mrpt::poses::CPose2D(100.0, 50.0, 0.5) +
           mrpt::poses::CPose2D(r * std::sin(a), r * (1 - std::cos(a)), a);
}

// A LiDAR odometry is the first pose source, so it defines {map}; it stalls
// across the reset, so the wheel odometry is the first source back after it.
void test_drifted_wheels_first_after_reset_motion()
{
    StateEstimationSmoother est;
    if (VERBOSE)
    {
        est.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    {
        auto cfg = mrpt::containers::yaml::FromText(CONFIG);
        est.initialize(cfg);
    }

    bool   resetDone = false;
    double maxErrXY  = 0;
    double maxErrYaw = 0;
    double tWheels   = 0;
    for (double t = 0; t <= T_END + 1e-9; t += POSE_PERIOD)
    {
        for (; tWheels <= t + 1e-9; tWheels += WHEELS_PERIOD)
        {
            mrpt::obs::CObservationOdometry o;
            o.timestamp   = mrpt::Clock::fromDouble(tWheels);
            o.sensorLabel = "odom_wheels";
            o.odometry    = wheels_pose(tWheels);
            est.fuse_odometry(o, o.sensorLabel);
        }
        if (!resetDone && t >= RESET_AT - 1e-9)
        {
            est.reset_motion();
            resetDone = true;
        }
        if (t < RESET_AT - 1e-9 || t >= POSE_RESUMES - 1e-9)
        {
            mrpt::poses::CPose3DPDFGaussian p;
            p.mean = mrpt::poses::CPose3D::FromXYZYawPitchRoll(V * t, 0, 0, 0, 0, 0);
            p.cov.setDiagonal(mrpt::square(0.01));
            est.fuse_pose(mrpt::Clock::fromDouble(t), p, "lidar_odom");
        }

        const auto ns = est.estimated_navstate(mrpt::Clock::fromDouble(t), "map");
        if (t > 1.0 && ns)
        {
            const double err = std::hypot(ns->pose.mean.x() - V * t, ns->pose.mean.y());
            maxErrXY         = std::max(maxErrXY, err);
            maxErrYaw        = std::max(maxErrYaw, std::abs(ns->pose.mean.yaw()));
            if (VERBOSE || err > 0.3)
            {
                std::cout << "t=" << t << " pose in {map}: " << ns->pose.mean.asString() << "\n";
            }
        }
    }
    std::cout << "[reset_motion, drifted wheels first] max error in {map}: " << maxErrXY << " m, "
              << mrpt::RAD2DEG(maxErrYaw) << " deg\n";

    // A wheel odometry frame restored where it was first aligned would put the
    // vehicle where the wheels alone take it from there: here, 1.6 m and 25 deg
    // away by the end. The heading margin is the false turn of the wheels while
    // they are the only source.
    ASSERT_LT_(maxErrXY, 0.3);
    ASSERT_LT_(maxErrYaw, mrpt::DEG2RAD(5.0));
}

}  // namespace

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;
    try
    {
        test_drifted_wheels_first_after_reset_motion();
        std::cout << "Test passed." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed:\n" << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
