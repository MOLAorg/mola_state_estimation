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
 * @file   test-imu-gyro-abrupt-rotation.cpp
 * @brief  Unit test: an abrupt rotation during a pose-fusion blackout (e.g. a
 *         failed ICP registration) is tracked when raw IMU gyro measurements
 *         are fused (imu_angular_velocity_sigma>0), and missed otherwise.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/bits_math.h>  // mrpt::square
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/random/RandomGenerators.h>

#include <cmath>
#include <iostream>
#include <string>

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr double T              = 0.1;  // time step [s]
constexpr size_t SETTLE_STEPS   = 15;  // steady, fully-observed steps before the turn
constexpr size_t BLACKOUT_STEPS = 6;  // steps with NO pose fusion (simulated ICP failure)
constexpr double WZ_TRUE        = 1.2;  // abrupt true yaw rate during the blackout [rad/s]

auto& rng = mrpt::random::getRandomGenerator();

std::string make_config(double gyroSigma)
{
    // predict_twist_filter disabled on purpose: this test isolates the raw
    // graph twist estimate itself (see test-predict-twist-filter.cpp for the
    // separate low-pass behavior).
    return std::string(R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 3.0
    max_time_to_use_velocity_model: 2.0
    min_time_difference_to_create_new_frame: 0.05
    sigma_random_walk_acceleration_linear: 1.0
    sigma_random_walk_acceleration_angular: 1.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.10
    sigma_twist_from_consecutive_poses_linear: 1.0
    sigma_twist_from_consecutive_poses_angular: 1.0
    estimate_geo_reference: false
    async_backend: false
    predict_twist_filter_enabled: false
    imu_normalized_gravity_alignment_sigma: 0
    imu_angular_velocity_sigma: )###") +
           std::to_string(gyroSigma) + "\n";
}

// Drives the estimator through a settled, fully pose-observed phase, then an
// abrupt yaw-rate change during which NO pose is fused (mirrors a failed ICP
// registration): only IMU gyro readings arrive, gated by `gyroSigma`. Returns
// the predicted yaw rate (twist.wz) at the end of the blackout.
double run_and_get_predicted_wz(double gyroSigma)
{
    // Fixed seed: the IMU noise below must not make this test flaky.
    rng.randomize(1);

    mola::state_estimation_smoother::StateEstimationSmoother est;
    est.initialize(mrpt::containers::yaml::FromText(make_config(gyroSigma)));

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
    double               t      = 0;

    const auto fuseImuWz = [&](double wz)
    {
        mrpt::obs::CObservationIMU obsImu;
        obsImu.timestamp   = mrpt::Clock::fromDouble(t);
        obsImu.sensorLabel = "imu";
        obsImu.set(mrpt::obs::IMU_WX, rng.drawGaussian1D(0.0, 0.01));
        obsImu.set(mrpt::obs::IMU_WY, rng.drawGaussian1D(0.0, 0.01));
        obsImu.set(mrpt::obs::IMU_WZ, wz + rng.drawGaussian1D(0.0, 0.01));
        est.fuse_imu(obsImu);
    };

    // Phase 1: settle with zero rotation, fully observed (pose + IMU each step).
    for (size_t i = 1; i <= SETTLE_STEPS; i++)
    {
        t = T * static_cast<double>(i);

        mrpt::poses::CPose3DPDFGaussian odo;
        odo.mean = gtPose;  // unchanged: zero motion
        odo.cov.setIdentity();
        odo.cov *= mrpt::square(0.02);
        est.fuse_pose(mrpt::Clock::fromDouble(t), odo, "odom");

        fuseImuWz(0.0);
    }

    // Phase 2: abrupt rotation starts, but NO pose is fused for BLACKOUT_STEPS
    // (as if ICP had failed and could not register any of these scans) --
    // only IMU gyro readings (if gyroSigma>0) carry the real motion.
    for (size_t i = 1; i <= BLACKOUT_STEPS; i++)
    {
        t += T;
        mrpt::math::CVectorFixedDouble<6> d;
        d.setZero();
        d[5]   = WZ_TRUE * T;  // yaw increment
        gtPose = gtPose + mrpt::poses::Lie::SE<3>::exp(d);

        fuseImuWz(WZ_TRUE);
    }

    const auto st = est.estimated_navstate(mrpt::Clock::fromDouble(t), "map");
    ASSERT_(st.has_value());

    if (VERBOSE)
    {
        std::cout << "[gyroSigma=" << gyroSigma << "] predicted wz=" << st->twist.wz
                  << " true wz=" << WZ_TRUE << "\n";
    }
    return st->twist.wz;
}

// With IMU gyro fusion enabled, the predictor should track the abrupt rotation
// even while starved of pose observations; without it, it should stay close to
// the pre-turn (zero) rate -- reproducing the ICP-prior staleness bug.
void test_gyro_tracks_abrupt_rotation_during_pose_blackout()
{
    const double wzWithGyro    = run_and_get_predicted_wz(0.05);
    const double wzWithoutGyro = run_and_get_predicted_wz(0.0);

    if (VERBOSE)
    {
        std::cout << "wzWithGyro=" << wzWithGyro << " wzWithoutGyro=" << wzWithoutGyro
                  << " true=" << WZ_TRUE << "\n";
    }

    // With gyro fusion: predicted rate should be close to the true abrupt rate.
    ASSERT_LT_(std::abs(wzWithGyro - WZ_TRUE), 0.3 * WZ_TRUE);

    // Without gyro fusion: predicted rate should still be close to the OLD
    // (pre-turn, zero) rate, i.e. far from the true one -- this is the bug
    // this factor fixes.
    ASSERT_LT_(std::abs(wzWithoutGyro), 0.3 * WZ_TRUE);

    // The gyro-fused prediction must track markedly better than without.
    ASSERT_LT_(std::abs(wzWithGyro - WZ_TRUE), 0.5 * std::abs(wzWithoutGyro - WZ_TRUE));
}

}  // namespace

int main()
{
    try
    {
        test_gyro_tracks_abrupt_rotation_during_pose_blackout();
        std::cout << "Test successful." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed with exception:\n" << e.what() << "\n";
        return 1;
    }
}
