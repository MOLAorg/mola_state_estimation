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
#include <mrpt/obs/CObservationIMU.h>
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
    relative_pose_increment_sigma_lin: 0.02
    relative_pose_increment_sigma_ang: 0.005
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

// The first warmupSteps poses are each followed by a solve, matching a front
// end that queries per observation, so the window reaches its steady state
// before anything is batched. After that, batchSize poses are queued per
// solve instead, the way the async backend defers process_pending_gtsam_updates()
// to whatever arrived since it last woke up.
void feed(
    mola::state_estimation_smoother::StateEstimationSmoother& nav, unsigned batchSize = 1,
    unsigned warmupSteps = 0)
{
    const auto cov = mrpt::math::CMatrixDouble66(mrpt::math::CMatrixDouble66::Identity() * 1e-4);

    for (unsigned i = 0; i < N_POSES; i++)
    {
        const double t = i * DT;
        const auto   p =
            mrpt::poses::CPose3D::FromXYZYawPitchRoll(SPEED * t, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg);
        nav.fuse_pose(mrpt::Clock::fromDouble(t), mrpt::poses::CPose3DPDFGaussian(p, cov), "odom");

        const bool doSolve =
            i < warmupSteps ? true : ((i - warmupSteps + 1) % batchSize == 0) || i + 1 == N_POSES;
        if (doSolve)
        {
            (void)nav.estimated_navstate(mrpt::Clock::fromDouble(t), "odom");
        }
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

// With the knob on, the poses of every pose keyframe are kept: those already
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

// Same as test_covers_the_whole_run, but after a warmup that lets the window
// (10 keyframes, sliding_window_length=1.0s, DT=0.1s) reach steady state,
// pairs of fuse_pose() calls are queued per solve instead of one -- the way
// the async backend batches whatever arrived since it last woke up. Each pair
// ages one already-solved, mature keyframe out of the window before the
// pair's own solve runs. Finalizing it right there, rather than after that
// solve's writeback, would record it at the value from one solve earlier,
// missing whatever the two new poses just contributed to it.
void test_covers_the_whole_run_without_intermediate_solves()
{
    mola::state_estimation_smoother::StateEstimationSmoother nav;
    nav.initialize(mrpt::containers::yaml::FromText(params_with(true)));
    feed(nav, /*batchSize=*/2, /*warmupSteps=*/20);

    const auto traj = nav.estimated_trajectory(
        mrpt::Clock::time_point::min(), mrpt::Clock::time_point::max(), "odom");
    ASSERT_(traj.has_value());
    ASSERT_GT_(traj->size(), N_POSES / 2);
    ASSERT_GT_(traj->size() * DT, 2.0);

    for (const auto& [t, p] : *traj)
    {
        const double tt = mrpt::Clock::toDouble(t);
        ASSERT_NEAR_(p.y, 0.0, 0.05);
        ASSERT_NEAR_(p.z, 0.0, 0.05);
        ASSERT_NEAR_(p.x, SPEED * tt, 0.05);
    }
}

// IMU readings between the poses create keyframes of their own (a reading
// usually arrives before the pose of its scan). Their attitude only comes from
// kinematics and gravity, so they are left out: every returned stamp must be a
// pose stamp.
void test_imu_keyframes_left_out()
{
    mola::state_estimation_smoother::StateEstimationSmoother nav;
    nav.initialize(mrpt::containers::yaml::FromText(
        params_with(true) +
        "    imu_min_sample_period: 0.0\n"
        "    imu_nearby_keyframe_stamp_tolerance: 0.01\n"
        "    imu_angular_velocity_sigma: 0.1\n"
        "    imu_normalized_gravity_alignment_sigma: 2.0\n"));

    const auto cov = mrpt::math::CMatrixDouble66(mrpt::math::CMatrixDouble66::Identity() * 1e-4);
    for (unsigned i = 0; i < N_POSES; i++)
    {
        const double t = i * DT;
        nav.fuse_pose(
            mrpt::Clock::fromDouble(t),
            mrpt::poses::CPose3DPDFGaussian(
                mrpt::poses::CPose3D::FromXYZYawPitchRoll(SPEED * t, 0, 0, 0, 0, 0), cov),
            "odom");

        mrpt::obs::CObservationIMU imu;
        imu.timestamp   = mrpt::Clock::fromDouble(t + 0.5 * DT);
        imu.sensorLabel = "imu";
        imu.set(mrpt::obs::IMU_X_ACC, 0.0);
        imu.set(mrpt::obs::IMU_Y_ACC, 0.0);
        imu.set(mrpt::obs::IMU_Z_ACC, 9.81);
        imu.set(mrpt::obs::IMU_WX, 0.0);
        imu.set(mrpt::obs::IMU_WY, 0.0);
        imu.set(mrpt::obs::IMU_WZ, 0.0);
        nav.fuse_imu(imu);

        (void)nav.estimated_navstate(mrpt::Clock::fromDouble(t), "odom");
    }

    // Exactly the pose keyframes, all of them:
    const auto traj = nav.estimated_trajectory(
        mrpt::Clock::time_point::min(), mrpt::Clock::time_point::max(), "odom");
    ASSERT_(traj.has_value());
    ASSERT_EQUAL_(traj->size(), N_POSES);
    unsigned i = 0;
    for (const auto& [t, p] : *traj)
    {
        ASSERT_NEAR_(mrpt::Clock::toDouble(t), i * DT, 1e-6);
        i++;
    }
}

// A pose arriving within min_time_difference_to_create_new_frame of an IMU
// keyframe reuses it: that keyframe holds the pose and is exported, at the
// stamp of the IMU reading that created it.
void test_reused_imu_keyframes_kept()
{
    mola::state_estimation_smoother::StateEstimationSmoother nav;
    nav.initialize(mrpt::containers::yaml::FromText(
        params_with(true) +
        "    imu_min_sample_period: 0.0\n"
        "    imu_nearby_keyframe_stamp_tolerance: 0.01\n"
        "    imu_angular_velocity_sigma: 0.1\n"
        "    imu_normalized_gravity_alignment_sigma: 2.0\n"));

    constexpr double T0     = 1.0;
    constexpr double IMU_DT = 0.005;
    const auto cov = mrpt::math::CMatrixDouble66(mrpt::math::CMatrixDouble66::Identity() * 1e-4);
    for (unsigned i = 0; i < N_POSES; i++)
    {
        const double t = T0 + i * DT;

        mrpt::obs::CObservationIMU imu;
        imu.timestamp   = mrpt::Clock::fromDouble(t - IMU_DT);
        imu.sensorLabel = "imu";
        imu.set(mrpt::obs::IMU_X_ACC, 0.0);
        imu.set(mrpt::obs::IMU_Y_ACC, 0.0);
        imu.set(mrpt::obs::IMU_Z_ACC, 9.81);
        imu.set(mrpt::obs::IMU_WX, 0.0);
        imu.set(mrpt::obs::IMU_WY, 0.0);
        imu.set(mrpt::obs::IMU_WZ, 0.0);
        nav.fuse_imu(imu);

        nav.fuse_pose(
            mrpt::Clock::fromDouble(t),
            mrpt::poses::CPose3DPDFGaussian(
                mrpt::poses::CPose3D::FromXYZYawPitchRoll(SPEED * (t - T0), 0, 0, 0, 0, 0), cov),
            "odom");

        (void)nav.estimated_navstate(mrpt::Clock::fromDouble(t), "odom");
    }

    // Every pose, at the stamp of the IMU keyframe it reused:
    const auto traj = nav.estimated_trajectory(
        mrpt::Clock::time_point::min(), mrpt::Clock::time_point::max(), "odom");
    ASSERT_(traj.has_value());
    ASSERT_EQUAL_(traj->size(), N_POSES);
    unsigned i = 0;
    for (const auto& [t, p] : *traj)
    {
        ASSERT_NEAR_(mrpt::Clock::toDouble(t), T0 + i * DT - IMU_DT, 1e-6);
        i++;
    }
}

// A reset() mid-run, as a front end does after a divergence, must not leave a
// gap: the keyframes still in the window are recorded as final, and the poses
// fused after it continue the trajectory.
void test_reset_keeps_the_window()
{
    mola::state_estimation_smoother::StateEstimationSmoother nav;
    nav.initialize(mrpt::containers::yaml::FromText(params_with(true)));

    const auto cov = mrpt::math::CMatrixDouble66(mrpt::math::CMatrixDouble66::Identity() * 1e-4);
    constexpr unsigned RESET_AT = N_POSES / 2;
    for (unsigned i = 0; i < N_POSES; i++)
    {
        if (i == RESET_AT)
        {
            nav.reset();
        }
        const double t = i * DT;
        const auto   p =
            mrpt::poses::CPose3D::FromXYZYawPitchRoll(SPEED * t, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg);
        nav.fuse_pose(mrpt::Clock::fromDouble(t), mrpt::poses::CPose3DPDFGaussian(p, cov), "odom");
        (void)nav.estimated_navstate(mrpt::Clock::fromDouble(t), "odom");
    }

    const auto traj = nav.estimated_trajectory(
        mrpt::Clock::time_point::min(), mrpt::Clock::time_point::max(), "odom");
    ASSERT_(traj.has_value());

    // No hole around the reset: consecutive stamps are at most one input step
    // apart (1 s of window, 10 keyframes, would be missing otherwise).
    double prevT = -1;
    for (const auto& [t, p] : *traj)
    {
        const double tt = mrpt::Clock::toDouble(t);
        if (prevT >= 0)
        {
            ASSERT_LT_(tt - prevT, 1.5 * DT);
        }
        ASSERT_NEAR_(p.x, SPEED * tt, 0.05);
        prevT = tt;
    }
    // From the first pose to the last one:
    ASSERT_NEAR_(mrpt::Clock::toDouble(traj->begin()->first), 0.0, 1e-6);
    ASSERT_GT_(mrpt::Clock::toDouble(traj->rbegin()->first), (RESET_AT + 1) * DT);
}

// With the first pose linked to the {map} origin, a reset_motion() mid-run
// must not link the first keyframe after it there too: earlier poses (in {map}
// or in an odometry frame) already placed the vehicle elsewhere, and the
// estimate must stay with them. A full reset() starts from scratch instead: the
// first pose after it is linked to the origin again, as needed when the front
// end redefines its frame.
void test_reset_mid_run(
    bool keepFrames, const std::string& poseFrame, bool newSource = false, bool twice = false)
{
    mola::state_estimation_smoother::StateEstimationSmoother nav;
    nav.initialize(mrpt::containers::yaml::FromText(
        params_with(true) + "    link_first_pose_to_reference_origin_sigma: 1e-6\n"));

    const auto cov = mrpt::math::CMatrixDouble66(mrpt::math::CMatrixDouble66::Identity() * 1e-4);
    constexpr unsigned RESET_AT = N_POSES / 2;
    for (unsigned i = 0; i < N_POSES; i++)
    {
        if (i == RESET_AT)
        {
            if (keepFrames)
            {
                nav.reset_motion();
                if (twice)
                {
                    // No solve in between: what was kept is not trusted.
                    nav.reset_motion();
                }
            }
            else
            {
                nav.reset();
            }
        }
        const double t = i * DT;
        // In {map}, which the first pose defines, the vehicle moves along its x
        // axis. An odometry frame starts elsewhere, so it is rotated and offset
        // with respect to {map}:
        const auto inMap =
            mrpt::poses::CPose3D::FromXYZYawPitchRoll(SPEED * t, 0, 0, 0.0_deg, 0.0_deg, 0.0_deg);
        const auto odomStart =
            mrpt::poses::CPose3D::FromXYZYawPitchRoll(5.0, -3.0, 0.5, 30.0_deg, 2.0_deg, -1.5_deg);
        const auto p = poseFrame == "map" ? inMap : odomStart + inMap;
        // A source that only starts after the reset, ahead of the kept one, in
        // a frame of its own: it must not redefine {map}.
        if (newSource && i >= RESET_AT)
        {
            const auto otherStart = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
                100.0, 50.0, 0.0, 90.0_deg, 0.0_deg, 0.0_deg);
            nav.fuse_pose(
                mrpt::Clock::fromDouble(t),
                mrpt::poses::CPose3DPDFGaussian(otherStart + inMap, cov), "other_odom");
        }
        nav.fuse_pose(
            mrpt::Clock::fromDouble(t), mrpt::poses::CPose3DPDFGaussian(p, cov), poseFrame);
        const auto ns = nav.estimated_navstate(mrpt::Clock::fromDouble(t), "map");
        if (i > RESET_AT + 1)
        {
            const double expectedX = keepFrames && !twice ? SPEED * t : SPEED * (t - RESET_AT * DT);
            ASSERT_(ns.has_value());
            ASSERT_NEAR_(ns->pose.mean.x(), expectedX, 0.05);
            ASSERT_NEAR_(ns->pose.mean.y(), 0.0, 0.05);
            ASSERT_NEAR_(ns->pose.mean.z(), 0.0, 0.05);
            ASSERT_NEAR_(ns->twist.vx, SPEED, 0.2);
        }
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
        {"covers_the_whole_run_without_intermediate_solves",
         &test_covers_the_whole_run_without_intermediate_solves},
        {"imu_keyframes_left_out", &test_imu_keyframes_left_out},
        {"reused_imu_keyframes_kept", &test_reused_imu_keyframes_kept},
        {"reset_keeps_the_window", &test_reset_keeps_the_window},
        {"reset_motion_keeps_frames", [] { test_reset_mid_run(true, "map"); }},
        {"reset_motion_keeps_frames_odom", [] { test_reset_mid_run(true, "odom"); }},
        {"reset_motion_keeps_frames_new_source", [] { test_reset_mid_run(true, "odom", true); }},
        {"reset_motion_twice_without_solve_is_full",
         [] { test_reset_mid_run(true, "odom", false, true); }},
        {"full_reset_redefines_frames", [] { test_reset_mid_run(false, "odom"); }},
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
