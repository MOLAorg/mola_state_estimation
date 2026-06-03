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
 * @file   test-state-estimation-simple.cpp
 * @brief  Unit tests for StateEstimationSimple
 * @author Jose Luis Blanco Claraco
 * @date   Dec 10, 2025
 */

#include <mola_state_estimation_simple/StateEstimationSimple.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/filesystem.h>

#include <algorithm>
#include <cmath>
#include <iostream>

using namespace std::string_literals;
using namespace mrpt::literals;

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

// Helper to create a configuration
mrpt::containers::yaml get_default_config()
{
    const char* yaml_text = R"###(
params:
    max_time_to_use_velocity_model: 2.0
    sigma_random_walk_acceleration_linear: 1.0
    sigma_random_walk_acceleration_angular: 1.0
    sigma_relative_pose_linear: 0.1
    sigma_relative_pose_angular: 0.1
    sigma_imu_angular_velocity: 0.05
    enforce_planar_motion: false
)###";
    return mrpt::containers::yaml::FromText(yaml_text);
}

// --------------------------------------------------------------------------
// Test 1: Pose Fusion & Twist Calculation
// --------------------------------------------------------------------------
// Feed two poses. The estimator should calculate the velocity (twist) between
// them and be able to extrapolate to a future time.
void test_pose_and_twist()
{
    std::cout << "[Test 1] Pose Fusion & Twist Calculation... ";

    mola::state_estimation_simple::StateEstimationSimple estimator;
    estimator.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    estimator.initialize(get_default_config());

    const double v_x = 2.0;  // m/s

    // t=0: Initial Pose at origin
    {
        auto pdf = mrpt::poses::CPose3DPDFGaussian(
            mrpt::poses::CPose3D::Identity(), mrpt::math::CMatrixDouble66::Identity());
        estimator.fuse_pose(mrpt::Clock::fromDouble(0.0), pdf, "map");
    }

    // t=1: Moved 2 meters in X
    {
        auto pdf = mrpt::poses::CPose3DPDFGaussian(
            mrpt::poses::CPose3D(v_x, 0, 0), mrpt::math::CMatrixDouble66::Identity());
        estimator.fuse_pose(mrpt::Clock::fromDouble(1.0), pdf, "map");
    }

    // Check 1: Twist should have been calculated
    auto twistOpt = estimator.get_last_twist();
    ASSERT_(twistOpt.has_value());
    ASSERT_NEAR_(twistOpt->vx, v_x, 1e-3);
    ASSERT_NEAR_(twistOpt->vy, 0.0, 1e-3);

    // Check 2: Extrapolation to t=1.5
    // Should be at x = 2.0 + (0.5 * 2.0) = 3.0
    auto stateOpt = estimator.estimated_navstate(mrpt::Clock::fromDouble(1.5), "map");
    ASSERT_(stateOpt.has_value());
    ASSERT_NEAR_(stateOpt->pose.mean.x(), 3.0, 1e-3);

    std::cout << "OK\n";
}

// --------------------------------------------------------------------------
// Test 2: Odometry Fusion
// --------------------------------------------------------------------------
// Initialize, then feed differential odometry.
void test_odometry_fusion()
{
    std::cout << "[Test 2] Odometry Fusion... ";

    mola::state_estimation_simple::StateEstimationSimple estimator;
    estimator.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    estimator.initialize(get_default_config());

    // 1. Must initialize with a known pose
    estimator.fuse_pose(
        mrpt::Clock::fromDouble(0.0),
        mrpt::poses::CPose3DPDFGaussian(
            mrpt::poses::CPose3D::Identity(), mrpt::math::CMatrixDouble66::Identity()),
        "map");

    // 2. Initialize Odometry baseline (t=0)
    {
        mrpt::obs::CObservationOdometry odom;
        odom.timestamp = mrpt::Clock::fromDouble(0.0);
        odom.odometry  = mrpt::poses::CPose2D::Identity();
        estimator.fuse_odometry(odom);
    }

    estimator.fuse_pose(
        mrpt::Clock::fromDouble(0.1),
        mrpt::poses::CPose3DPDFGaussian(
            mrpt::poses::CPose3D::Identity(), mrpt::math::CMatrixDouble66::Identity()),
        "map");

    // 3. Feed Odometry increment (t=1)
    // Robot moved 1m in X according to wheel encoders
    {
        mrpt::obs::CObservationOdometry odom;
        odom.timestamp = mrpt::Clock::fromDouble(1.0);
        odom.odometry  = mrpt::poses::CPose2D(1.0, 0, 0);  // 1m forward

        // Could be: estimator.fuse_odometry(odom);
        // But let's test the RawDataConsumer API:
        estimator.onNewObservation(std::make_shared<const mrpt::obs::CObservationOdometry>(odom));
    }

    // Check: Current state should reflect the odometry increment
    // Note: StateEstimationSimple logic sums the increment to the last pose.
    auto stateOpt = estimator.estimated_navstate(mrpt::Clock::fromDouble(1.0), "map");
    ASSERT_(stateOpt.has_value());
    ASSERT_NEAR_(stateOpt->pose.mean.x(), 1.0, 1e-3);

    std::cout << "OK\n";
}

// --------------------------------------------------------------------------
// Test 3: IMU Fusion (Angular Velocity)
// --------------------------------------------------------------------------
// The simple estimator uses IMU to overwrite the angular velocity (twist)
// for extrapolation.
void test_imu_angular_velocity()
{
    std::cout << "[Test 3] IMU Angular Velocity... ";

    mola::state_estimation_simple::StateEstimationSimple estimator;
    estimator.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    estimator.initialize(get_default_config());

    // Initialize at origin
    estimator.fuse_pose(
        mrpt::Clock::fromDouble(0.0),
        mrpt::poses::CPose3DPDFGaussian(
            mrpt::poses::CPose3D::Identity(), mrpt::math::CMatrixDouble66::Identity()),
        "map");

    // Force a twist via IMU at t=0.1
    // Rotating around Z at 1.0 rad/s
    {
        mrpt::obs::CObservationIMU imu;
        imu.timestamp = mrpt::Clock::fromDouble(0.1);
        imu.set(mrpt::obs::IMU_WX, 0.0);
        imu.set(mrpt::obs::IMU_WY, 0.0);
        imu.set(mrpt::obs::IMU_WZ, 1.0);  // 1 rad/s
        imu.sensorPose = mrpt::poses::CPose3D::Identity();  // IMU aligned with body

        // Could be also: estimator.fuse_imu(imu);
        estimator.onNewObservation(std::make_shared<const mrpt::obs::CObservationIMU>(imu));
    }

    // Check if twist was updated
    auto twist = estimator.get_last_twist();
    ASSERT_(twist.has_value());
    ASSERT_NEAR_(twist->wz, 1.0, 1e-3);

    // Predict state at t=1.1 (delta = 1.0s from last *twist* update?
    // Wait, estimated_navstate calculates dt from last_pose_obs_tim (t=0.0)
    // So dt = 1.1s.
    // Rotation = w * dt = 1.0 * 1.1 = 1.1 rad.
    auto stateOpt = estimator.estimated_navstate(mrpt::Clock::fromDouble(1.1), "map");

    ASSERT_(stateOpt.has_value());

    double y, p, r;
    stateOpt->pose.mean.getYawPitchRoll(y, p, r);

    // Logic check: The class uses the *last stored twist* to extrapolate from *last stored pose*.
    // Last pose t=0.0. Last twist is the one set by IMU. Target t=1.1. dt=1.1.
    // Expected yaw = 1.1 rad.
    ASSERT_NEAR_(y, 1.1, 1e-3);

    std::cout << "OK\n";
}

// --------------------------------------------------------------------------
// Test 4: Planar Motion Enforcement
// --------------------------------------------------------------------------
// Verify that Z, Pitch, and Roll are zeroed out if config requests it.
void test_planar_motion()
{
    std::cout << "[Test 4] Planar Motion Constraint... ";

    mola::state_estimation_simple::StateEstimationSimple estimator;
    auto                                                 cfg = get_default_config();
    cfg["params"]["enforce_planar_motion"]                   = true;
    estimator.initialize(cfg);

    // Feed a non-planar pose (Z height, Roll, Pitch)
    auto nonPlanarPose = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        1.0, 2.0, 5.0,  // X, Y, Z=5
        1.0, 0.5, 0.2);  // Yaw, Pitch=0.5, Roll=0.2

    estimator.fuse_pose(
        mrpt::Clock::fromDouble(0.0),
        mrpt::poses::CPose3DPDFGaussian(nonPlanarPose, mrpt::math::CMatrixDouble66::Identity()),
        "map");
    estimator.fuse_pose(
        mrpt::Clock::fromDouble(0.05),
        mrpt::poses::CPose3DPDFGaussian(nonPlanarPose, mrpt::math::CMatrixDouble66::Identity()),
        "map");

    // Check immediate estimate
    auto stateOpt = estimator.estimated_navstate(mrpt::Clock::fromDouble(0.1), "map");

    ASSERT_(stateOpt.has_value());

    // Z should be 0, Pitch 0, Roll 0. X/Y/Yaw should remain (approximately).
    const auto& p = stateOpt->pose.mean;
    ASSERT_NEAR_(p.z(), 0.0, 1e-5);

    double y, pit, rol;
    p.getYawPitchRoll(y, pit, rol);
    ASSERT_NEAR_(pit, 0.0, 1e-5);
    ASSERT_NEAR_(rol, 0.0, 1e-5);
    ASSERT_NEAR_(y, 1.0, 1e-5);  // Yaw should be preserved

    std::cout << "OK\n";
}

// --------------------------------------------------------------------------
// Test 5: GNSS Ignored
// --------------------------------------------------------------------------
// Verify that GNSS messages don't crash the system (explicitly ignored in source).
void test_gnss_ignored()
{
    std::cout << "[Test 5] GNSS Ignored... ";

    mola::state_estimation_simple::StateEstimationSimple estimator;
    estimator.initialize(get_default_config());

    // Init
    estimator.fuse_pose(
        mrpt::Clock::fromDouble(0.0),
        mrpt::poses::CPose3DPDFGaussian(
            mrpt::poses::CPose3D::Identity(), mrpt::math::CMatrixDouble66::Identity()),
        "map");
    estimator.fuse_pose(
        mrpt::Clock::fromDouble(0.05),
        mrpt::poses::CPose3DPDFGaussian(
            mrpt::poses::CPose3D::Identity(), mrpt::math::CMatrixDouble66::Identity()),
        "map");

    // Feed GNSS
    mrpt::obs::CObservationGPS gps;
    gps.timestamp = mrpt::Clock::fromDouble(0.5);
    // Just ensure it doesn't throw
    try
    {
        estimator.fuse_gnss(gps);
    }
    catch (...)
    {
        std::cerr << "fuse_gnss threw an exception!\n";
        throw;
    }

    // State should remain unchanged/valid
    auto stateOpt = estimator.estimated_navstate(mrpt::Clock::fromDouble(0.0), "map");
    ASSERT_(stateOpt.has_value());

    std::cout << "OK\n";
}

// --------------------------------------------------------------------------
// Test 6: CObservationRobotPose via onNewObservation
// --------------------------------------------------------------------------
// Verify that incoming CObservationRobotPose observations are dispatched to
// fuse_pose(), and that a non-identity sensorPose is correctly compensated.
void test_robot_pose_observation()
{
    std::cout << "[Test 6] CObservationRobotPose dispatch... ";

    mola::state_estimation_simple::StateEstimationSimple estimator;
    estimator.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    estimator.initialize(get_default_config());

    const auto cov = mrpt::math::CMatrixDouble66::Identity();

    // t=0: Initial pose at origin via CObservationRobotPose
    {
        auto obs         = mrpt::obs::CObservationRobotPose::Create();
        obs->timestamp   = mrpt::Clock::fromDouble(0.0);
        obs->sensorLabel = "lidar_odom";
        obs->pose        = mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D::Identity(), cov);
        estimator.onNewObservation(obs);
    }

    // t=1: Sensor reports pose (3,0,0) but the sensor is mounted 1m forward
    // of the vehicle frame. Vehicle pose should therefore be (2,0,0).
    {
        auto obs         = mrpt::obs::CObservationRobotPose::Create();
        obs->timestamp   = mrpt::Clock::fromDouble(1.0);
        obs->sensorLabel = "lidar_odom";
        obs->sensorPose  = mrpt::poses::CPose3D(1.0, 0, 0, 0, 0, 0);
        obs->pose        = mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D(3.0, 0, 0), cov);
        estimator.onNewObservation(obs);
    }

    // Twist should reflect 2 m/s in X (vehicle frame went 0 -> 2 in 1 s)
    auto twistOpt = estimator.get_last_twist();
    ASSERT_(twistOpt.has_value());
    ASSERT_NEAR_(twistOpt->vx, 2.0, 1e-3);

    // Extrapolation to t=1.5 should give vehicle x = 2 + 0.5*2 = 3.0
    auto stateOpt = estimator.estimated_navstate(mrpt::Clock::fromDouble(1.5), "map");
    ASSERT_(stateOpt.has_value());
    ASSERT_NEAR_(stateOpt->pose.mean.x(), 3.0, 1e-3);

    std::cout << "OK\n";
}

// --------------------------------------------------------------------------
// Test 7: Mixed ICP + 3-D wheel-odometry source
// --------------------------------------------------------------------------
// LiDAR ICP supplies the ground-truth trajectory via fuse_pose() at 1 Hz.
// Wheel odometry arrives as CObservationRobotPose at 2 Hz and advances the
// state between ICP scans via fuse_odometry_3d_pose().
//
// Key invariants checked:
//  (a) The twist derived from two consecutive ICP poses is the true ICP-to-ICP
//      velocity and is NOT contaminated by odometry deltas applied to
//      state_.last_pose between those scans.
//  (b) estimated_navstate() extrapolates correctly from the odom-advanced pose
//      while an ICP scan is still pending.
//  (c) estimated_navstate() extrapolates correctly from the ICP pose once a
//      new scan has been fused.
void test_icp_and_3d_odometry_fusion()
{
    std::cout << "[Test 7] ICP + 3-D odometry fusion... ";

    mola::state_estimation_simple::StateEstimationSimple estimator;
    estimator.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    estimator.initialize(get_default_config());

    const auto   cov = mrpt::math::CMatrixDouble66::Identity();
    const double v_x = 1.0;  // m/s — robot moves steadily in +X

    // Helper: inject a CObservationRobotPose for the wheel-odometry source.
    auto send_wheel_odom = [&](double t, double x)
    {
        auto obs         = mrpt::obs::CObservationRobotPose::Create();
        obs->timestamp   = mrpt::Clock::fromDouble(t);
        obs->sensorLabel = "wheel_odom";
        obs->pose        = mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D(x, 0, 0), cov);
        estimator.onNewObservation(obs);
    };

    // --- t=0: both sources initialise at origin ---
    estimator.fuse_pose(
        mrpt::Clock::fromDouble(0.0),
        mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D::Identity(), cov), "map");
    send_wheel_odom(0.0, 0.0);

    // --- t=0.5: odom fires mid-scan, advancing state_.last_pose to x=0.5 ---
    send_wheel_odom(0.5, 0.5);

    // (b) Mid-scan extrapolation: odom twist (1 m/s) from (0.5,0,0) at t=0.5
    //     queried at t=0.7 → expected x = 0.5 + 1.0*0.2 = 0.7
    {
        auto s = estimator.estimated_navstate(mrpt::Clock::fromDouble(0.7), "map");
        ASSERT_(s.has_value());
        ASSERT_NEAR_(s->pose.mean.x(), 0.7, 1e-3);
    }

    // --- t=1: LiDAR ICP delivers the true vehicle position (1,0,0) ---
    estimator.fuse_pose(
        mrpt::Clock::fromDouble(1.0),
        mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D(v_x, 0, 0), cov), "map");

    // (a) Twist must equal the ICP-to-ICP velocity (1.0 m/s).
    //     A regression would give 0.5 m/s because the broken code used the
    //     odom-advanced state_.last_pose=(0.5,0,0) as the base instead of the
    //     per-source ICP pose=(0,0,0).
    {
        auto tw = estimator.get_last_twist();
        ASSERT_(tw.has_value());
        ASSERT_NEAR_(tw->vx, v_x, 1e-3);
    }

    // (c) Post-ICP extrapolation: ICP pose (1,0,0) at t=1, query at t=1.5
    //     → expected x = 1.0 + 1.0*0.5 = 1.5
    {
        auto s = estimator.estimated_navstate(mrpt::Clock::fromDouble(1.5), "map");
        ASSERT_(s.has_value());
        ASSERT_NEAR_(s->pose.mean.x(), 1.5, 1e-3);
    }

    std::cout << "OK\n";
}

// --------------------------------------------------------------------------
// Test 8: Velocity Kalman filter
// --------------------------------------------------------------------------
// The filter should smooth pose-derived velocities. We feed three consecutive
// ICP pose pairs whose finite-difference velocities are 2.0, 3.0, 2.0 m/s.
// The middle "spike" (3.0) must be attenuated, and the estimate must then
// converge back toward 2.0 after the consistent third measurement.
//
// Parameters chosen so that smoothing is clearly visible:
//   sigma_relative_pose_linear = 1.0  -> R = 1.0  (m/s)^2 at dt=1 s
//   sigma_random_walk_acceleration_linear = 0.1  -> process noise = 0.01/step
//
// Analytic values for the linear vx component (dt=1 s throughout):
//   After t=1 (bootstrap): vx = 2.0, P = 1.0
//   After t=2 (raw 3.0):   P_pred=1.01, K=0.5025, vx=2.502, P=0.502
//   After t=3 (raw 2.0):   P_pred=0.512, K=0.339, vx=2.332
void test_velocity_kalman_filter()
{
    std::cout << "[Test 8] Velocity Kalman filter... ";

    const char* yaml_text = R"###(
params:
    max_time_to_use_velocity_model: 5.0
    sigma_random_walk_acceleration_linear: 0.1
    sigma_random_walk_acceleration_angular: 0.1
    sigma_relative_pose_linear: 1.0
    sigma_relative_pose_angular: 1.0
    sigma_imu_angular_velocity: 0.05
    velocity_filter_enabled: true
    enforce_planar_motion: false
)###";

    // ---- Test A: filter ON -----------------------------------------------
    {
        mola::state_estimation_simple::StateEstimationSimple est;
        est.initialize(mrpt::containers::yaml::FromText(yaml_text));

        const auto cov = mrpt::math::CMatrixDouble66::Identity();

        // t=0: origin
        est.fuse_pose(
            mrpt::Clock::fromDouble(0.0),
            mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D::Identity(), cov), "map");

        // t=1: x=2 -> raw vx=2.0, bootstrap
        est.fuse_pose(
            mrpt::Clock::fromDouble(1.0),
            mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D(2.0, 0, 0), cov), "map");

        ASSERT_(est.get_last_twist().has_value());
        const double vx_after_t1 = est.get_last_twist()->vx;
        ASSERT_NEAR_(vx_after_t1, 2.0, 1e-6);

        // t=2: x=5 -> raw vx=3.0, spike -- filtered result must be strictly
        // between 2.0 and 3.0.
        est.fuse_pose(
            mrpt::Clock::fromDouble(2.0),
            mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D(5.0, 0, 0), cov), "map");

        ASSERT_(est.get_last_twist().has_value());
        const double vx_after_spike = est.get_last_twist()->vx;
        ASSERT_(vx_after_spike > 2.0);
        ASSERT_(vx_after_spike < 3.0);
        // Expected ~2.502 -- check with generous tolerance.
        ASSERT_NEAR_(vx_after_spike, 2.502, 0.01);

        // t=3: x=7 -> raw vx=2.0 -- filter must start converging back below the
        // spike value.
        est.fuse_pose(
            mrpt::Clock::fromDouble(3.0),
            mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D(7.0, 0, 0), cov), "map");

        ASSERT_(est.get_last_twist().has_value());
        const double vx_converging = est.get_last_twist()->vx;
        ASSERT_(vx_converging < vx_after_spike);
        ASSERT_(vx_converging > 2.0);
    }

    // ---- Test B: filter OFF (baseline -- raw finite-difference) ----------
    {
        const char*                                          yaml_no_filter = R"###(
params:
    max_time_to_use_velocity_model: 5.0
    sigma_random_walk_acceleration_linear: 0.1
    sigma_random_walk_acceleration_angular: 0.1
    sigma_relative_pose_linear: 1.0
    sigma_relative_pose_angular: 1.0
    sigma_imu_angular_velocity: 0.05
    velocity_filter_enabled: false
    enforce_planar_motion: false
)###";
        mola::state_estimation_simple::StateEstimationSimple est;
        est.initialize(mrpt::containers::yaml::FromText(yaml_no_filter));

        const auto cov = mrpt::math::CMatrixDouble66::Identity();

        est.fuse_pose(
            mrpt::Clock::fromDouble(0.0),
            mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D::Identity(), cov), "map");
        est.fuse_pose(
            mrpt::Clock::fromDouble(1.0),
            mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D(2.0, 0, 0), cov), "map");
        est.fuse_pose(
            mrpt::Clock::fromDouble(2.0),
            mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D(5.0, 0, 0), cov), "map");

        // Without filter the spike is passed through unchanged.
        ASSERT_(est.get_last_twist().has_value());
        ASSERT_NEAR_(est.get_last_twist()->vx, 3.0, 1e-6);
    }

    std::cout << "OK\n";
}

// --------------------------------------------------------------------------
// Test 9: Multi-rate interleaved IMU + lagging LiDAR poses
// --------------------------------------------------------------------------
// Reproduces the real LIO timing: a high-rate IMU with near-real-time stamps,
// interleaved with lower-rate LiDAR poses whose (mid-scan, post-ICP) stamps lag
// the IMU stream. The poses encode a constant 1 m/s motion plus an alternating
// per-scan offset, so the raw finite-difference velocity oscillates 0.5<->1.5.
//
// With the filter OFF the oscillation passes straight through. With the filter
// ON and per-component clocks, the linear velocity is (a) actually updated --
// the lagging poses are NOT rejected as backwards-in-time against the
// IMU-advanced clock, which used to starve the linear velocity to zero -- and
// (b) smoothed toward the true 1 m/s.
//
// Returns the max |vx - 1.0| over the second half of the run.
double run_interleaved_linear_velocity(bool filter_enabled)
{
    const std::string yaml_text =
        "params:\n"
        "    max_time_to_use_velocity_model: 5.0\n"
        "    sigma_random_walk_acceleration_linear: 0.5\n"
        "    sigma_random_walk_acceleration_angular: 0.5\n"
        "    sigma_relative_pose_linear: 0.1\n"
        "    sigma_relative_pose_angular: 0.1\n"
        "    sigma_imu_angular_velocity: 0.05\n"
        "    velocity_filter_enabled: "s +
        (filter_enabled ? "true" : "false") +
        "\n"
        "    enforce_planar_motion: false\n";

    mola::state_estimation_simple::StateEstimationSimple est;
    est.initialize(mrpt::containers::yaml::FromText(yaml_text));

    const auto   cov    = mrpt::math::CMatrixDouble66::Identity();
    const double v_true = 1.0;  // [m/s] true forward speed
    const double dt     = 0.1;  // [s]   LiDAR period
    const double offset = 0.025;  // [m]   alternating pose noise -> +-0.5 m/s raw
    const int    N      = 40;

    double max_dev = 0.0;
    for (int k = 0; k <= N; k++)
    {
        const double t_pose = k * dt;

        // Fresh IMU sample slightly AHEAD of the pose stamp, so the pose is "in
        // the past" w.r.t. the IMU clock (the starvation trigger). IMU carries
        // angular velocity only.
        mrpt::obs::CObservationIMU imu;
        imu.timestamp = mrpt::Clock::fromDouble(t_pose + 0.5 * dt);
        imu.set(mrpt::obs::IMU_WX, 0.0);
        imu.set(mrpt::obs::IMU_WY, 0.0);
        imu.set(mrpt::obs::IMU_WZ, 0.0);
        est.fuse_imu(imu);

        // LiDAR pose (lagging stamp): constant 1 m/s + alternating offset.
        const double x = v_true * t_pose + ((k % 2 == 0) ? offset : -offset);
        est.fuse_pose(
            mrpt::Clock::fromDouble(t_pose),
            mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D(x, 0, 0), cov), "map");

        // Assess after warm-up (second half of the run).
        if (k > N / 2)
        {
            const auto tw = est.get_last_twist();
            if (tw.has_value())
            {
                max_dev = std::max(max_dev, std::abs(tw->vx - v_true));
            }
        }
    }
    return max_dev;
}

void test_multirate_interleaved_velocity()
{
    std::cout << "[Test 9] Multi-rate interleaved IMU + lagging poses... ";

    const double dev_on  = run_interleaved_linear_velocity(true);
    const double dev_off = run_interleaved_linear_velocity(false);

    if (VERBOSE)
    {
        std::cout << "\n  max|vx-1| filter ON=" << dev_on << " OFF=" << dev_off << "\n";
    }

    // Filter ON: linear velocity tracks ~1 m/s (smoothed), and crucially is NOT
    // starved to ~0 despite the IMU stamps being ahead of the pose stamps.
    // (A regression to a single shared clock would drop the lagging pose updates
    //  and leave vx ~ 0, i.e. dev_on ~ 1.0, failing this assertion.)
    ASSERT_LT_(dev_on, 0.2);

    // Filter OFF: the raw alternating velocity (+-0.5 m/s) passes through, so the
    // filter is doing real work here.
    ASSERT_GT_(dev_off, 0.4);

    // And ON must be clearly better than OFF.
    ASSERT_LT_(dev_on, 0.5 * dev_off);

    std::cout << "OK\n";
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_pose_and_twist();
        test_odometry_fusion();
        test_imu_angular_velocity();
        test_planar_motion();
        test_gnss_ignored();
        test_robot_pose_observation();
        test_icp_and_3d_odometry_fusion();
        test_velocity_kalman_filter();
        test_multirate_interleaved_velocity();

        std::cout << "\nAll StateEstimationSimple tests passed!\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "\nERROR: " << e.what() << std::endl;
        return 1;
    }
}