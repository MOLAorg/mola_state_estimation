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
 * @file   test-prediction-residual-seed.cpp
 * @brief  A start-up transient must not set the prediction covariance taken
 *         from an odometry source's own prediction errors.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/poses/CPose3D.h>

#include <cmath>
#include <iostream>

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr double      T         = 0.1;  // scan period [s]
constexpr size_t      SKIPPED   = 5;  // scans after the first one that are not fused
constexpr double      V         = 13.0;  // forward speed, from the very start [m/s]
constexpr size_t      NUM_STEPS = 36;
constexpr const char* ODOM      = "odom";

constexpr auto CONFIG = R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    max_time_to_use_velocity_model: 2.0
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 3.0
    relative_pose_increment_sigma_lin: 0.02
    relative_pose_increment_sigma_ang: 0.005
    min_time_difference_to_create_new_frame: 0.05
    sigma_random_walk_acceleration_linear: 1.0
    sigma_random_walk_acceleration_angular: 10.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.10
    sigma_twist_from_consecutive_poses_linear: 1.0
    sigma_twist_from_consecutive_poses_angular: 1.0
    estimate_geo_reference: false
    async_backend: false
    predict_twist_filter_enabled: true
    predict_twist_filter_time_const: 0.3
    predict_covariance_from_residuals: true
    predict_residual_time_const: 10.0
)###";

void fuse_odom(mola::state_estimation_smoother::StateEstimationSmoother& est, const double t)
{
    mrpt::poses::CPose3DPDFGaussian odo;
    odo.mean = mrpt::poses::CPose3D::FromXYZYawPitchRoll(V * t, 0, 0, 0, 0, 0);
    odo.cov.setDiagonal(mrpt::square(0.02));
    est.fuse_pose(mrpt::Clock::fromDouble(t), odo, ODOM);
}

// The source moves at V from the start, while the estimator starts at rest; its
// first readings are then far from any prediction. Once it has been seen moving
// steadily for a while, its one-step prediction must be about as tight as its
// actual prediction errors, not as loose as that start.
void test_startup_transient_does_not_set_prediction_cov()
{
    mola::state_estimation_smoother::StateEstimationSmoother est;
    {
        auto cfg = mrpt::containers::yaml::FromText(CONFIG);
        est.initialize(cfg);
    }

    // Anchor {map} with a tight prior:
    {
        auto cov = mrpt::math::CMatrixDouble66::Identity();
        cov *= 1e-6;
        est.fuse_pose(
            mrpt::Clock::fromDouble(0),
            mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D::Identity(), cov),
            est.parameters().reference_frame_name);
    }

    // As a front end does: ask for the prediction at each scan, then fuse the
    // scan's pose, except for a few scans right after the first one.
    for (size_t i = 0; i < NUM_STEPS; i++)
    {
        const double t = T * static_cast<double>(i);
        if (i > 0)
        {
            [[maybe_unused]] const auto pred =
                est.estimated_navstate(mrpt::Clock::fromDouble(t), ODOM);
        }
        if (i == 0 || i > SKIPPED)
        {
            fuse_odom(est, t);
        }
    }

    const double tq = T * static_cast<double>(NUM_STEPS);
    const auto   st = est.estimated_navstate(mrpt::Clock::fromDouble(tq), ODOM);
    ASSERT_(st.has_value());

    const auto   cov    = st->pose.cov_inv.inverse_LLt();
    const double sigmaX = std::sqrt(cov(0, 0));
    const double errX   = std::abs(st->pose.mean.x() - V * tq);

    if (VERBOSE)
    {
        std::cout << "one-step prediction: sigma_x=" << sigmaX << " m, error=" << errX
                  << " m, twist vx=" << st->twist.vx << "\n";
    }

    // The prediction itself is good by now:
    ASSERT_LT_(errX, 0.1);
    // ...and so must be its stated uncertainty (a front end discards a motion
    // model looser than ~1 m):
    ASSERT_LT_(sigmaX, 0.5);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_startup_transient_does_not_set_prediction_cov();
        std::cout << "Test successful." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
