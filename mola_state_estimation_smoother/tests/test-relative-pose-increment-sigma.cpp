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
 * @file   test-relative-pose-increment-sigma.cpp
 * @brief  Verifies relative_pose_increment_sigma_* replaces the covariance a
 *         drifting pose source supplies, for its increment factors.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <Eigen/Eigenvalues>
#include <cmath>
#include <iostream>
#include <string>

using namespace std::string_literals;

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

// A source that publishes the covariance of its ABSOLUTE dead-reckoned pose:
// a large, slowly growing number that says nothing about one increment. This
// is what a legged or wheeled platform's own estimator typically reports.
constexpr double SOURCE_SIGMA = 1.0;  // [m]
// What the platform's kinematics actually support per increment.
constexpr double INCREMENT_SIGMA = 0.015;  // [m]

constexpr double POSE_DT      = 0.05;  // [s]  (20 Hz)
constexpr double DURATION     = 3.0;  // [s]
constexpr double VELOCITY_X   = 1.0;  // [m/s]
constexpr size_t NUM_READINGS = static_cast<size_t>(DURATION / POSE_DT) + 1;

std::string params_yaml(double incrementSigmaLin)
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
    relative_factors_frame_ids_re: "odom_wheels"
    relative_pose_increment_sigma_lin: )###" +
        std::to_string(incrementSigmaLin) + R"###(
    relative_pose_increment_sigma_ang: 0.0
)###";
}

/// Returns the reported position sigma at the last keyframe. `correlated`
/// makes the source covariance carry the strong x-y and x-yaw correlations an
/// absolute dead-reckoned pose typically has.
double run(double incrementSigmaLin, bool correlated = false)
{
    mola::state_estimation_smoother::StateEstimationSmoother est;
    if (VERBOSE)
    {
        est.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    est.initialize(mrpt::containers::yaml::FromText(params_yaml(incrementSigmaLin)));

    mrpt::Clock::time_point lastStamp;
    for (size_t i = 0; i < NUM_READINGS; i++)
    {
        const double t     = POSE_DT * static_cast<double>(i);
        const auto   stamp = mrpt::Clock::fromDouble(t);
        lastStamp          = stamp;

        mrpt::poses::CPose3DPDFGaussian pdf;
        pdf.mean =
            mrpt::poses::CPose3D::FromXYZYawPitchRoll(VELOCITY_X * t, 0.0, 0.0, 0.0, 0.0, 0.0);
        for (int k = 0; k < 3; k++)
        {
            pdf.cov(k, k) = SOURCE_SIGMA * SOURCE_SIGMA;
        }
        for (int k = 3; k < 6; k++)
        {
            pdf.cov(k, k) = mrpt::square(0.05);
        }
        if (correlated)
        {
            // indices: x,y,z,yaw,pitch,roll
            pdf.cov(0, 1) = pdf.cov(1, 0) = 0.9 * SOURCE_SIGMA * SOURCE_SIGMA;
            pdf.cov(0, 3) = pdf.cov(3, 0) = 0.8 * SOURCE_SIGMA * 0.05;
            pdf.cov(1, 3) = pdf.cov(3, 1) = 0.7 * SOURCE_SIGMA * 0.05;
            // A valid covariance, so any failure is the estimator's:
            ASSERT_GT_(pdf.cov.asEigen().eigenvalues().real().minCoeff(), 0.0);
        }
        est.fuse_pose(stamp, pdf, "odom_wheels");
    }

    const auto navOpt = est.estimated_navstate(lastStamp, "map");
    ASSERT_(navOpt.has_value());
    // The estimator reports an information matrix, not a covariance.
    mrpt::math::CMatrixDouble66 cov;
    navOpt->pose.getCovariance(cov);
    return std::sqrt(cov(0, 0));
}

void run_test()
{
    // Off: the increment factors carry the source's own 1 m sigma.
    const double sigmaSourceCov = run(0.0);
    // On: the increments assert what the platform actually supports.
    const double sigmaAsserted = run(INCREMENT_SIGMA);

    std::cout << "reported sigma_x, source covariance: " << sigmaSourceCov << " m\n";
    std::cout << "reported sigma_x, asserted 15 mm:    " << sigmaAsserted << " m\n";

    ASSERT_(std::isfinite(sigmaSourceCov));
    ASSERT_(std::isfinite(sigmaAsserted));

    // Asserting a tight per-increment accuracy must make the graph's own
    // opinion of the trajectory correspondingly tighter. Without the override
    // the same chain of increments is nearly uninformative.
    ASSERT_LT_(sigmaAsserted, sigmaSourceCov);
    ASSERTMSG_(
        sigmaAsserted * 3 < sigmaSourceCov, "The override must dominate, not merely nudge: got " +
                                                std::to_string(sigmaAsserted) + " against " +
                                                std::to_string(sigmaSourceCov));

    // Sanity: the asserted value should land in the same order of magnitude as
    // what was asserted, not collapse to zero or stay at the source's number.
    ASSERT_LT_(sigmaAsserted, 10 * INCREMENT_SIGMA);

    // A correlated source covariance: the override must drop the source's
    // cross terms along with its diagonal. Keeping them next to a much smaller
    // diagonal gives an indefinite matrix, which the solver cannot factor.
    const double sigmaCorrelated = run(INCREMENT_SIGMA, true);
    std::cout << "reported sigma_x, asserted 15 mm, correlated source: " << sigmaCorrelated
              << " m\n";
    ASSERT_(std::isfinite(sigmaCorrelated));
    ASSERT_LT_(std::abs(sigmaCorrelated - sigmaAsserted), 0.5 * sigmaAsserted);
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
