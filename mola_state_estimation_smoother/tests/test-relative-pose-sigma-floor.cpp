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
 * @file   test-relative-pose-sigma-floor.cpp
 * @brief  Unit tests for sigma_relative_pose_{linear,angular}
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

constexpr double      T         = 0.1;  // time step [s]
constexpr size_t      NUM_STEPS = 40;
constexpr double      V0        = 1.0;  // forward velocity [m/s]
constexpr const char* ODOM      = "odom";

// When `withFloorKeys` is false the two keys are omitted entirely, which is
// what a config file written before this feature existed looks like.
std::string make_config(double sigmaLin, double sigmaAng, bool withFloorKeys = true)
{
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
)###") + (withFloorKeys ? ("    sigma_relative_pose_linear: " + std::to_string(sigmaLin) +
                           "\n    sigma_relative_pose_angular: " + std::to_string(sigmaAng) + "\n")
                        : std::string());
}

struct Sample
{
    mrpt::math::CMatrixDouble66 cov;
    mrpt::poses::CPose3D        mean;
};

// Drives the estimator with a well-conditioned constant-velocity motion, which
// is the regime where the graph's own marginal is tightest and therefore where
// the floor has the most to do. Returns the pose PDF from every successful
// estimated_navstate() query in the {map} frame.
std::vector<Sample> run_and_collect(double sigmaLin, double sigmaAng, bool withFloorKeys = true)
{
    mola::state_estimation_smoother::StateEstimationSmoother est;
    {
        auto cfg = mrpt::containers::yaml::FromText(make_config(sigmaLin, sigmaAng, withFloorKeys));
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

    mrpt::poses::CPose3D gtPose = mrpt::poses::CPose3D::Identity();
    std::vector<Sample>  out;

    for (size_t i = 1; i <= NUM_STEPS; i++)
    {
        const double t = T * static_cast<double>(i);

        mrpt::math::CVectorFixedDouble<6> d;
        d.setZero();
        d[0]   = V0 * T;
        gtPose = gtPose + mrpt::poses::Lie::SE<3>::exp(d);

        {
            mrpt::math::TTwist3D tw;
            tw.vx = V0;
            mrpt::math::CMatrixDouble66 twCov;
            twCov.setIdentity();
            twCov *= mrpt::square(0.05);
            est.fuse_twist(mrpt::Clock::fromDouble(t), tw, twCov);
        }
        {
            // A deliberately confident pose source, so the marginal the graph
            // reports is far tighter than any floor under test.
            mrpt::poses::CPose3DPDFGaussian odo;
            odo.mean = gtPose;
            odo.cov.setDiagonal(mrpt::square(0.001));
            est.fuse_pose(mrpt::Clock::fromDouble(t), odo, ODOM);
        }

        // Query slightly ahead of the last fused stamp, i.e. the extrapolating
        // regime a front end actually uses.
        const auto st = est.estimated_navstate(
            mrpt::Clock::fromDouble(t + 0.5 * T), est.parameters().reference_frame_name);
        if (st)
        {
            Sample s;
            s.cov  = st->pose.cov_inv.inverse_LLt();
            s.mean = st->pose.mean;
            out.push_back(s);
        }
    }
    return out;
}

// The shipped default must reproduce the pre-existing behavior exactly. Tested
// the way it is actually at risk: a config file that predates this feature does
// not mention the two keys at all, and must be bit-identical to one that sets
// them to their defaults.
void test_default_is_inert()
{
    const auto absent    = run_and_collect(0.0, 0.0, false);
    const auto explicit0 = run_and_collect(0.0, 0.0, true);

    ASSERT_(absent.size() > 5);
    ASSERT_EQUAL_(absent.size(), explicit0.size());

    for (size_t k = 0; k < absent.size(); k++)
    {
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                ASSERT_EQUAL_(absent[k].cov(i, j), explicit0[k].cov(i, j));
            }
        }
    }

    // And the un-floored run really is tighter than the floors swept below, so
    // the tests that follow are measuring something.
    ASSERT_LT_(std::sqrt(absent.back().cov(0, 0)), 0.5);
}

// The floor must add exactly sigma^2 to each diagonal entry of its block, and
// touch nothing else: not the mean, not the off-diagonals, not the other block.
void test_floor_adds_exactly_its_variance()
{
    constexpr double SIG_LIN = 0.5;
    constexpr double SIG_ANG = 0.1;

    const auto base    = run_and_collect(0.0, 0.0);
    const auto floored = run_and_collect(SIG_LIN, SIG_ANG);

    ASSERT_EQUAL_(base.size(), floored.size());
    ASSERT_(!base.empty());

    for (size_t k = 0; k < base.size(); k++)
    {
        // The mean is untouched: this is a weight, not a correction.
        ASSERT_NEAR_((base[k].mean - floored[k].mean).translation().norm(), 0.0, 1e-9);

        for (int i = 0; i < 6; i++)
        {
            const double expectedAdd = (i < 3) ? mrpt::square(SIG_LIN) : mrpt::square(SIG_ANG);
            ASSERT_NEAR_(floored[k].cov(i, i) - base[k].cov(i, i), expectedAdd, 1e-9);

            for (int j = 0; j < 6; j++)
            {
                if (i == j)
                {
                    continue;
                }
                ASSERT_NEAR_(floored[k].cov(i, j), base[k].cov(i, j), 1e-9);
            }
        }
    }

    if (VERBOSE)
    {
        std::cout << "base sigma_x=" << std::sqrt(base.back().cov(0, 0))
                  << " floored sigma_x=" << std::sqrt(floored.back().cov(0, 0)) << "\n";
    }
}

// The floor's whole purpose: bound how confident the estimator is allowed to
// report itself, however well conditioned the window is.
void test_floor_bounds_reported_confidence()
{
    constexpr double SIG_LIN = 0.5;

    const auto base    = run_and_collect(0.0, 0.0);
    const auto floored = run_and_collect(SIG_LIN, 0.0);
    ASSERT_(!base.empty());

    for (size_t k = 0; k < floored.size(); k++)
    {
        ASSERT_GE_(std::sqrt(floored[k].cov(0, 0)), SIG_LIN);
    }
    // ... and it really was doing something: the un-floored run is tighter.
    ASSERT_LT_(std::sqrt(base.back().cov(0, 0)), SIG_LIN);
}

// A single isotropic variance added to all three entries of a block commutes
// with any permutation of them, which is what makes the floor immune to the
// Euler-vs-Lie ordering the consumers of this covariance disagree about.
void test_floor_is_isotropic_per_block()
{
    const auto floored = run_and_collect(0.5, 0.1);
    ASSERT_(!floored.empty());

    const auto base = run_and_collect(0.0, 0.0);
    for (size_t k = 0; k < floored.size(); k++)
    {
        const double addX = floored[k].cov(0, 0) - base[k].cov(0, 0);
        const double addY = floored[k].cov(1, 1) - base[k].cov(1, 1);
        const double addZ = floored[k].cov(2, 2) - base[k].cov(2, 2);
        ASSERT_NEAR_(addX, addY, 1e-12);
        ASSERT_NEAR_(addX, addZ, 1e-12);

        const double addR = floored[k].cov(3, 3) - base[k].cov(3, 3);
        const double addP = floored[k].cov(4, 4) - base[k].cov(4, 4);
        const double addW = floored[k].cov(5, 5) - base[k].cov(5, 5);
        ASSERT_NEAR_(addR, addP, 1e-12);
        ASSERT_NEAR_(addR, addW, 1e-12);
    }
}

}  // namespace

int main()
{
    try
    {
        test_default_is_inert();
        test_floor_adds_exactly_its_variance();
        test_floor_bounds_reported_confidence();
        test_floor_is_isotropic_per_block();
        std::cout << "Test successful." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed with exception:\n" << e.what() << "\n";
        return 1;
    }
}
