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
 * @file   test-relative-pose-source-cov-translation.cpp
 * @brief  With relative_pose_increment_from_source_covariance, the option
 *         relative_pose_increment_source_covariance_translation_only keeps the
 *         reading's translation covariance but the configured rotation sigma.
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

// Configured per-increment sigmas: loose translation, tight rotation.
constexpr double CONFIG_SIGMA_LIN = 0.05;  // [m]
constexpr double CONFIG_SIGMA_ANG = 1e-4;  // [rad]
// Covariance the source reports with each reading: tight translation, loose
// rotation.
constexpr double SOURCE_SIGMA_LIN = 0.002;  // [m]
constexpr double SOURCE_SIGMA_ANG = 0.02;  // [rad]

constexpr double POSE_DT      = 0.05;  // [s]
constexpr double DURATION     = 3.0;  // [s]
constexpr double VELOCITY_X   = 1.0;  // [m/s]
constexpr size_t NUM_READINGS = static_cast<size_t>(DURATION / POSE_DT) + 1;

struct Sigmas
{
    double x   = 0;
    double yaw = 0;
};

Sigmas run(bool fromSource, bool translationOnly)
{
    const std::string yaml =
        std::string(R"###(
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
)###") +
        "    relative_pose_increment_sigma_lin: " + std::to_string(CONFIG_SIGMA_LIN) +
        "\n    relative_pose_increment_sigma_ang: " + std::to_string(CONFIG_SIGMA_ANG) +
        "\n    relative_pose_increment_from_source_covariance: " + (fromSource ? "true" : "false") +
        "\n    relative_pose_increment_source_covariance_translation_only: " +
        (translationOnly ? "true" : "false") + "\n";

    mola::state_estimation_smoother::StateEstimationSmoother est;
    if (VERBOSE)
    {
        est.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    est.initialize(mrpt::containers::yaml::FromText(yaml));

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
            pdf.cov(k, k) = mrpt::square(SOURCE_SIGMA_LIN);
        }
        for (int k = 3; k < 6; k++)
        {
            pdf.cov(k, k) = mrpt::square(SOURCE_SIGMA_ANG);
        }
        est.fuse_pose(stamp, pdf, "odom");
    }

    const auto navOpt = est.estimated_navstate(lastStamp, "map");
    ASSERT_(navOpt.has_value());
    mrpt::math::CMatrixDouble66 cov;
    navOpt->pose.getCovariance(cov);  // x,y,z,yaw,pitch,roll
    Sigmas s;
    s.x   = std::sqrt(cov(0, 0));
    s.yaw = std::sqrt(cov(3, 3));
    return s;
}

void run_test()
{
    const Sigmas fixedSigmas   = run(false, false);
    const Sigmas fullSource    = run(true, false);
    const Sigmas translationSc = run(true, true);
    const Sigmas ignored       = run(false, true);

    std::cout << "fixed sigmas:            sigma_x " << fixedSigmas.x << " m, sigma_yaw "
              << fixedSigmas.yaw << " rad\n";
    std::cout << "full source covariance:  sigma_x " << fullSource.x << " m, sigma_yaw "
              << fullSource.yaw << " rad\n";
    std::cout << "source translation only: sigma_x " << translationSc.x << " m, sigma_yaw "
              << translationSc.yaw << " rad\n";

    for (const auto& s : {fixedSigmas, fullSource, translationSc, ignored})
    {
        ASSERT_(std::isfinite(s.x));
        ASSERT_(std::isfinite(s.yaw));
    }

    // Translation follows the source, as with the full covariance:
    ASSERT_LT_(std::abs(translationSc.x - fullSource.x), 0.3 * fullSource.x);
    ASSERT_GT_(fixedSigmas.x, 3 * translationSc.x);

    // Rotation follows the configured sigma, not the source's:
    ASSERT_LT_(translationSc.yaw, 0.2 * fullSource.yaw);
    ASSERT_LT_(std::abs(translationSc.yaw - fixedSigmas.yaw), 0.3 * fixedSigmas.yaw);

    // Without relative_pose_increment_from_source_covariance it does nothing:
    ASSERT_EQUAL_(ignored.x, fixedSigmas.x);
    ASSERT_EQUAL_(ignored.yaw, fixedSigmas.yaw);
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
