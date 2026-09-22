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
 * @file   test-relative-pose-increment-growth.cpp
 * @brief  Verifies relative_pose_increment_sigma_per_sqrt_meter: the asserted
 *         uncertainty grows with motion, stays at the floor while stationary,
 *         and adds up to the same total whatever the increment rate.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <cmath>
#include <iostream>
#include <string>

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr double SOURCE_DT = 0.01;  // [s]  (100 Hz)
constexpr double DURATION  = 3.0;  // [s]
constexpr double FLOOR     = 1e-3;  // [m]
constexpr double K_SQRT_M  = 0.05;  // [m/sqrt(m)]

// Just under 20 Hz and 5 Hz, so the kept readings land on a fixed grid that
// includes the last one: the final query then extrapolates nothing.
constexpr double PERIOD_FINE   = 0.045;  // [s]
constexpr double PERIOD_COARSE = 0.195;  // [s]

// The kinematic factors are made loose, so the reported uncertainty is that of
// the relative chain itself: a random walk of variance K^2 per meter.
std::string params_yaml(double samplePeriod, double kSqrtM)
{
    return R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    link_first_pose_to_reference_origin_sigma: 1e-6
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 10.0
    max_time_to_use_velocity_model: 2.0
    sigma_random_walk_acceleration_linear: 100.0
    sigma_random_walk_acceleration_angular: 100.0
    sigma_integrator_position: 100.0
    sigma_integrator_orientation: 100.0
    estimate_geo_reference: false
    relative_factors_frame_ids_re: "legged_odom"
    relative_pose_increment_sigma_lin: )###" +
           std::to_string(FLOOR) + R"###(
    relative_pose_increment_sigma_ang: 1e-3
    relative_pose_increment_sigma_per_sqrt_meter: )###" +
           std::to_string(kSqrtM) + R"###(
    pose_min_sample_period: )###" +
           std::to_string(samplePeriod) + "\n";
}

/// Returns the reported x sigma at the end of the run.
double run(double samplePeriod, double kSqrtM, double velocity)
{
    mola::state_estimation_smoother::StateEstimationSmoother est;
    if (VERBOSE)
    {
        est.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    est.initialize(mrpt::containers::yaml::FromText(params_yaml(samplePeriod, kSqrtM)));

    const auto              n = static_cast<size_t>(std::lround(DURATION / SOURCE_DT));
    mrpt::Clock::time_point lastStamp;
    for (size_t i = 0; i <= n; i++)
    {
        const double t = SOURCE_DT * static_cast<double>(i);
        lastStamp      = mrpt::Clock::fromDouble(t);

        mrpt::poses::CPose3DPDFGaussian pdf;
        pdf.mean = mrpt::poses::CPose3D::FromXYZYawPitchRoll(velocity * t, 0.0, 0.0, 0.0, 0.0, 0.0);
        for (int k = 0; k < 6; k++)
        {
            pdf.cov(k, k) = mrpt::square(1e-3);
        }
        est.fuse_pose(lastStamp, pdf, "legged_odom");
    }

    const auto nav = est.estimated_navstate(lastStamp, "map");
    ASSERT_(nav.has_value());
    mrpt::math::CMatrixDouble66 cov;
    nav->pose.getCovariance(cov);
    return std::sqrt(cov(0, 0));
}

void run_test()
{
    constexpr double V = 1.0;  // [m/s]

    // Growth: moving, with and without the random-walk term.
    const double sigmaFlat   = run(PERIOD_FINE, 0.0, V);
    const double sigmaGrowth = run(PERIOD_FINE, K_SQRT_M, V);
    // Floor: standing still, the growth term must add nothing.
    const double sigmaStillFlat   = run(PERIOD_FINE, 0.0, 0.0);
    const double sigmaStillGrowth = run(PERIOD_FINE, K_SQRT_M, 0.0);
    // Rate independence: the same path split into 4x fewer increments.
    const double sigmaCoarse = run(PERIOD_COARSE, K_SQRT_M, V);

    const double expected = K_SQRT_M * std::sqrt(V * DURATION);

    std::cout << "moving, flat:        " << sigmaFlat << " m\n"
              << "moving, growth:      " << sigmaGrowth << " m (expected ~" << expected << ")\n"
              << "still, flat:         " << sigmaStillFlat << " m\n"
              << "still, growth:       " << sigmaStillGrowth << " m\n"
              << "moving, growth, 5Hz: " << sigmaCoarse << " m\n";

    ASSERT_GT_(sigmaGrowth, 5 * sigmaFlat);
    ASSERT_LT_(std::abs(sigmaGrowth - expected), 0.25 * expected);
    ASSERT_LT_(std::abs(sigmaStillGrowth - sigmaStillFlat), 0.1 * sigmaStillFlat + 1e-6);

    // A sigma proportional to the increment size (rather than a variance)
    // would halve here, since the coarse run has 4x fewer, 4x longer steps.
    ASSERT_LT_(std::abs(sigmaCoarse - sigmaGrowth), 0.15 * sigmaGrowth);
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
