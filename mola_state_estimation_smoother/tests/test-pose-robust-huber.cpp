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
 * @file   test-pose-robust-huber.cpp
 * @brief  Verifies pose_robust_huber_threshold bounds the damage of a single
 *         gross error in a relative dead-reckoning source.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <cmath>
#include <iostream>
#include <string>

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr double ODOM_DT    = 0.05;  // [s]  (20 Hz)
constexpr double MAP_DT     = 0.20;  // [s]  (5 Hz)
constexpr double DURATION   = 3.0;  // [s]
constexpr double VELOCITY_X = 1.0;  // [m/s]
constexpr double SLIP_TIME  = 1.5;  // [s]
constexpr double SLIP       = 0.5;  // [m] a slipped foot, never undone
constexpr double MAP_SIGMA  = 0.05;  // [m]

std::string params_yaml(double huber)
{
    return R"###(
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
    relative_factors_frame_ids_re: "legged_odom"
    relative_pose_increment_sigma_lin: 0.01
    relative_pose_increment_sigma_ang: 0.005
    pose_robust_huber_threshold: )###" +
           std::to_string(huber) + "\n";
}

/// Returns the final position error against the truth.
double run(double huber)
{
    mola::state_estimation_smoother::StateEstimationSmoother est;
    if (VERBOSE)
    {
        est.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    est.initialize(mrpt::containers::yaml::FromText(params_yaml(huber)));

    const auto makePdf = [](double x, double sigma)
    {
        mrpt::poses::CPose3DPDFGaussian pdf;
        pdf.mean = mrpt::poses::CPose3D::FromXYZYawPitchRoll(x, 0.0, 0.0, 0.0, 0.0, 0.0);
        for (int k = 0; k < 3; k++)
        {
            pdf.cov(k, k) = sigma * sigma;
        }
        for (int k = 3; k < 6; k++)
        {
            pdf.cov(k, k) = mrpt::square(0.01);
        }
        return pdf;
    };

    const auto              n    = static_cast<size_t>(std::lround(DURATION / ODOM_DT));
    const auto              step = static_cast<size_t>(std::lround(MAP_DT / ODOM_DT));
    mrpt::Clock::time_point lastStamp;
    for (size_t i = 0; i <= n; i++)
    {
        const double t = ODOM_DT * static_cast<double>(i);
        lastStamp      = mrpt::Clock::fromDouble(t);

        const double trueX = VELOCITY_X * t;
        const double odomX = trueX + (t >= SLIP_TIME ? SLIP : 0.0);

        if (i % step == 0)
        {
            est.fuse_pose(lastStamp, makePdf(trueX, MAP_SIGMA), "map");
            (void)est.estimated_navstate(lastStamp, "map");
        }
        // The source's absolute covariance is irrelevant: the increments
        // assert relative_pose_increment_sigma_*.
        est.fuse_pose(lastStamp, makePdf(odomX, 1.0), "legged_odom");
    }

    const auto nav = est.estimated_navstate(lastStamp, "map");
    ASSERT_(nav.has_value());
    return std::abs(nav->pose.mean.x() - VELOCITY_X * DURATION);
}

void run_test()
{
    const double errGaussian = run(0.0);
    const double errHuber    = run(1.345);

    std::cout << "final error, Gaussian: " << errGaussian << " m\n"
              << "final error, Huber:    " << errHuber << " m\n";

    // Under a Gaussian the slip, asserted at 50 sigmas, drags the trajectory
    // against the map poses; Huber gives it a bounded influence.
    ASSERT_LT_(errHuber, 0.5 * errGaussian);
    ASSERT_LT_(errHuber, 2 * MAP_SIGMA);
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
