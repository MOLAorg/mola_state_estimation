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
 * @file   test-finalized-trajectory.cpp
 * @brief  Unit tests for StateEstimationSmoother::estimated_trajectory()
 * @author Jose Luis Blanco Claraco
 * @date   Sep 19, 2026
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/system/os.h>

#include <Eigen/Dense>  // required by mrpt's matrix accessors
#include <cmath>
#include <functional>
#include <iostream>
#include <map>
#include <string>

using namespace mrpt::literals;  // _deg

namespace
{
// Same configuration twice, differing only in the knob under test, so a
// difference between the two runs can only come from it.
std::string params_with(bool keepTrajectory)
{
    return std::string(R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    max_time_to_use_velocity_model: 2.0
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 1.0
    min_time_difference_to_create_new_frame: 0.01
    sigma_random_walk_acceleration_linear: 1.0
    sigma_random_walk_acceleration_angular: 1.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.10
    initial_twist_sigma_lin: 20
    initial_twist_sigma_ang: 3
    estimate_geo_reference: false
    keep_finalized_trajectory: )###") +
           (keepTrajectory ? "true" : "false") + "\n";
}

// A straight run along +x at 1 m/s, one pose every 100 ms.
constexpr double   SPEED   = 1.0;
constexpr double   DT      = 0.1;
constexpr unsigned N_POSES = 50;

void feed(mola::state_estimation_smoother::StateEstimationSmoother& nav)
{
    const auto cov = mrpt::math::CMatrixDouble66(mrpt::math::CMatrixDouble66::Identity() * 1e-4);

    for (unsigned i = 0; i < N_POSES; i++)
    {
        const double t = i * DT;
        const auto   p =
            mrpt::poses::CPose3D::FromXYZYawPitchRoll(SPEED * t, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg);
        nav.fuse_pose(mrpt::Clock::fromDouble(t), mrpt::poses::CPose3DPDFGaussian(p, cov), "odom");

        // A front end queries the estimator once per observation; that is what
        // runs the solve whose values are written back into the keyframes.
        (void)nav.estimated_navstate(mrpt::Clock::fromDouble(t), "odom");
    }
}
}  // namespace

// The accessor stays silent unless asked for, since in a live system nothing
// reads it and the container would grow with the run.
void test_disabled_by_default()
{
    mola::state_estimation_smoother::StateEstimationSmoother nav;
    nav.initialize(mrpt::containers::yaml::FromText(params_with(false)));
    feed(nav);

    const auto traj = nav.estimated_trajectory(
        mrpt::Clock::time_point::min(), mrpt::Clock::time_point::max(), "odom");
    ASSERT_(!traj.has_value());
}

// With the knob on, the poses of every keyframe are kept: those already
// marginalized out, at their final optimized value, plus the ones still inside
// the window.
void test_covers_the_whole_run()
{
    mola::state_estimation_smoother::StateEstimationSmoother nav;
    nav.initialize(mrpt::containers::yaml::FromText(params_with(true)));
    feed(nav);

    const auto traj = nav.estimated_trajectory(
        mrpt::Clock::time_point::min(), mrpt::Clock::time_point::max(), "odom");
    ASSERT_(traj.has_value());
    ASSERT_GT_(traj->size(), N_POSES / 2);

    // The window is 1 s against a 4.9 s run, so most keyframes must have left
    // it: without the recording at marginalization, only the tail would remain.
    ASSERT_GT_(traj->size() * DT, 2.0);

    double prevT = -1, prevX = -1;
    for (const auto& [t, p] : *traj)
    {
        const double tt = mrpt::Clock::toDouble(t);
        ASSERT_GT_(tt, prevT);  // strictly increasing stamps
        ASSERT_GE_(p.x, prevX);  // monotone along the direction of travel
        // The input is a straight line with 1 cm sigmas: the optimum cannot be
        // far from it in any direction.
        ASSERT_NEAR_(p.y, 0.0, 0.05);
        ASSERT_NEAR_(p.z, 0.0, 0.05);
        ASSERT_NEAR_(p.x, SPEED * tt, 0.05);
        prevT = tt;
        prevX = p.x;
    }
}

// An unknown frame cannot be served, and must not be silently answered in the
// reference frame.
void test_unknown_frame()
{
    mola::state_estimation_smoother::StateEstimationSmoother nav;
    nav.initialize(mrpt::containers::yaml::FromText(params_with(true)));
    feed(nav);

    const auto traj = nav.estimated_trajectory(
        mrpt::Clock::time_point::min(), mrpt::Clock::time_point::max(), "no_such_frame");
    ASSERT_(!traj.has_value());
}

int main(int argc, char** argv)
{
    const std::map<std::string, std::function<void()>> tests = {
        {"disabled_by_default", &test_disabled_by_default},
        {"covers_the_whole_run", &test_covers_the_whole_run},
        {"unknown_frame", &test_unknown_frame},
    };

    bool anyFail = false;
    for (const auto& [name, f] : tests)
    {
        try
        {
            std::cout << "[ " << name << " ] Running..." << std::endl;
            f();
            std::cout << "[ " << name << " ] OK." << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cout << "[ " << name << " ] ERROR: " << std::endl << e.what() << std::endl;
            anyFail = true;
        }
    }
    (void)argc;
    (void)argv;
    return anyFail ? 1 : 0;
}
