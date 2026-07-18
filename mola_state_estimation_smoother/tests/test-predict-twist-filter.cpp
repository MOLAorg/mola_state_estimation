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
 * @file   test-predict-twist-filter.cpp
 * @brief  Unit tests for the predict-twist low-pass (predict_twist_filter_*)
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/Lie/SE.h>

#include <cmath>
#include <iostream>
#include <vector>

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr double      T          = 0.1;  // time step [s] (10 Hz keyframes)
constexpr size_t      NUM_STEPS  = 60;  // steps to drive the estimator
constexpr double      V0         = 0.8;  // nominal forward velocity [m/s]
constexpr double      JITTER_AMP = 0.4;  // square-wave velocity jitter amplitude [m/s]
constexpr const char* ODOM       = "odom";

std::string make_config(bool filterEnabled)
{
    // Loose angular sigma on purpose, so the boundary-node velocity is free to
    // track the injected jitter (this is exactly the regime the filter targets).
    return std::string(R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    max_time_to_use_velocity_model: 2.0
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 3.0
    min_time_difference_to_create_new_frame: 0.05
    sigma_random_walk_acceleration_linear: 1.0
    sigma_random_walk_acceleration_angular: 10.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.10
    sigma_twist_from_consecutive_poses_linear: 1.0
    sigma_twist_from_consecutive_poses_angular: 1.0
    estimate_geo_reference: false
    async_backend: false
    predict_twist_filter_enabled: )###") +
           (filterEnabled ? "true" : "false") +
           R"###(
    predict_twist_filter_time_const: 0.3
)###";
}

// Drives the estimator with a nominally-constant forward motion whose fused
// velocity carries a square-wave jitter, and returns the sequence of predicted
// forward velocities (twist.vx) from estimated_navstate() in the {map} frame.
std::vector<double> run_and_collect_predicted_vx(bool filterEnabled)
{
    mola::state_estimation_smoother::StateEstimationSmoother est;
    {
        auto cfg = mrpt::containers::yaml::FromText(make_config(filterEnabled));
        est.initialize(cfg);
    }

    // Anchor {map} to {odom} with a tight prior.
    {
        auto cov = mrpt::math::CMatrixDouble66::Identity();
        cov *= 1e-6;
        est.fuse_pose(
            mrpt::Clock::fromDouble(0),
            mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D::Identity(), cov),
            est.parameters().reference_frame_name);
    }

    mrpt::poses::CPose3D gtPose = mrpt::poses::CPose3D::Identity();

    std::vector<double> predictedVx;

    for (size_t i = 1; i <= NUM_STEPS; i++)
    {
        const double t    = T * static_cast<double>(i);
        const double sign = (i % 2 == 0) ? +1.0 : -1.0;
        const double vx   = V0 + sign * JITTER_AMP;  // jittery fused velocity

        // Advance the ground-truth pose by the nominal (un-jittered) motion.
        mrpt::math::CVectorFixedDouble<6> d;
        d.setZero();
        d[0]   = V0 * T;
        gtPose = gtPose + mrpt::poses::Lie::SE<3>::exp(d);

        // Fuse the jittery velocity as a twist observation.
        {
            mrpt::math::TTwist3D tw;
            tw.vx = vx;
            mrpt::math::CMatrixDouble66 twCov;
            twCov.setIdentity();
            twCov *= mrpt::square(0.05);
            est.fuse_twist(mrpt::Clock::fromDouble(t), tw, twCov);
        }

        // Fuse an odometry pose so the graph stays well-constrained.
        {
            mrpt::poses::CPose3DPDFGaussian odo;
            odo.mean = gtPose;
            odo.cov.setDiagonal(mrpt::square(0.02));
            est.fuse_pose(mrpt::Clock::fromDouble(t), odo, ODOM);
        }

        const auto st = est.estimated_navstate(
            mrpt::Clock::fromDouble(t), est.parameters().reference_frame_name);
        if (st)
        {
            predictedVx.push_back(st->twist.vx);
        }
    }

    if (VERBOSE)
    {
        std::cout << "[filter=" << (filterEnabled ? "on" : "off") << "] collected "
                  << predictedVx.size() << " predictions\n";
    }
    return predictedVx;
}

// Total-variation roughness: sum of |v[k] - v[k-1]|.
double roughness(const std::vector<double>& v)
{
    double r = 0;
    for (size_t k = 1; k < v.size(); k++)
    {
        r += std::abs(v[k] - v[k - 1]);
    }
    return r;
}

double mean(const std::vector<double>& v)
{
    double s = 0;
    for (const double x : v)
    {
        s += x;
    }
    return v.empty() ? 0.0 : s / static_cast<double>(v.size());
}

// The filter must smooth the predicted velocity and track its mean without bias.
void test_filter_smooths_and_tracks()
{
    const auto vxOff = run_and_collect_predicted_vx(false);
    const auto vxOn  = run_and_collect_predicted_vx(true);

    ASSERT_(vxOff.size() > 5);
    ASSERT_EQUAL_(vxOff.size(), vxOn.size());

    const double rOff = roughness(vxOff);
    const double rOn  = roughness(vxOn);

    if (VERBOSE)
    {
        std::cout << "roughness off=" << rOff << " on=" << rOn << " mean off=" << mean(vxOff)
                  << " on=" << mean(vxOn) << "\n";
    }

    // The low-pass must cut the step-to-step jitter by a clear margin. If the
    // same-stamp (dt<=0) history-preservation regressed, the filter would keep
    // resetting to the raw boundary twist and this margin would collapse.
    ASSERT_LT_(rOn, 0.6 * rOff);

    // ...without biasing the mean forward velocity away from the true V0.
    ASSERT_LT_(std::abs(mean(vxOn) - V0), 0.15);
}

// The filter is a plain deterministic EMA: identical inputs -> identical output.
void test_filter_is_deterministic()
{
    const auto a = run_and_collect_predicted_vx(true);
    const auto b = run_and_collect_predicted_vx(true);
    ASSERT_EQUAL_(a.size(), b.size());
    for (size_t k = 0; k < a.size(); k++)
    {
        ASSERT_EQUAL_(a[k], b[k]);
    }
}

}  // namespace

int main()
{
    try
    {
        test_filter_smooths_and_tracks();
        test_filter_is_deterministic();
        std::cout << "Test successful." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed with exception:\n" << e.what() << "\n";
        return 1;
    }
}
