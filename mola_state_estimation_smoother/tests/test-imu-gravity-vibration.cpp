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
 * @file   test-imu-gravity-vibration.cpp
 * @brief  A vehicle driving on flat ground with a vibrating IMU: the accelerometer
 *         direction oscillates by several degrees, while LiDAR poses are level and
 *         accurate. The short-term predicted attitude must stay level, i.e. the gravity
 *         factors must not drag the newest (IMU) keyframes' roll and pitch around.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random/RandomGenerators.h>

#include <cmath>
#include <iostream>
#include <string>

namespace
{
constexpr double LIDAR_PERIOD    = 0.1;  // [s]
constexpr double IMU_PERIOD      = 0.01;  // [s]
constexpr double DURATION        = 30.0;  // [s]
constexpr double SPEED           = 0.5;  // [m/s]
constexpr double VIB_AMPLITUDE   = 6.0;  // [deg] accelerometer tilt oscillation
constexpr double VIB_FREQ        = 7.3;  // [Hz]
constexpr double VIB_NOISE       = 3.0;  // [deg] per sample, on top
constexpr double PREDICT_AHEAD   = 0.15;  // [s] like a pose published at "now"
constexpr double MAX_TILT_ERR    = 1.0;  // [deg]
constexpr double GRAVITY         = 9.81;
constexpr double GYRO_NOISE      = 0.05;  // [rad/s]
constexpr double WARMUP_DURATION = 3.0;  // [s]

auto& rng = mrpt::random::getRandomGenerator();

// Shipped smoother defaults, with a synchronous backend so the test is
// deterministic, plus the gravity factor settings under test.
std::string make_config(double gravitySigmaDeg, double gravityPeriod)
{
    return R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 6.0
    relative_pose_increment_sigma_lin: 0.02
    relative_pose_increment_sigma_ang: 0.005
    max_time_to_use_velocity_model: 0.75
    min_time_difference_to_create_new_frame: 0.01
    imu_nearby_keyframe_stamp_tolerance: 0.10
    imu_min_sample_period: 0.1
    sigma_random_walk_acceleration_linear: 0.5
    sigma_random_walk_acceleration_angular: 10.0
    sigma_integrator_position: 0.1
    sigma_integrator_orientation: 1.0
    imu_angular_velocity_sigma: 0.1
    estimate_geo_reference: false
    async_backend: false
    imu_normalized_gravity_alignment_sigma: )###" +
           std::to_string(gravitySigmaDeg) +
           "\n    imu_gravity_min_sample_period: " + std::to_string(gravityPeriod) + "\n";
}

// Returns the worst roll/pitch [deg] of the predicted pose after the warm-up.
double run_session(double gravitySigmaDeg, double gravityPeriod)
{
    rng.randomize(1234);

    mola::state_estimation_smoother::StateEstimationSmoother est;
    est.initialize(mrpt::containers::yaml::FromText(make_config(gravitySigmaDeg, gravityPeriod)));

    double       maxTilt    = 0;
    const size_t imuPerScan = static_cast<size_t>(std::lround(LIDAR_PERIOD / IMU_PERIOD));
    const size_t nScans     = static_cast<size_t>(std::lround(DURATION / LIDAR_PERIOD));

    for (size_t k = 0; k < nScans; k++)
    {
        for (size_t i = 0; i < imuPerScan; i++)
        {
            const double t = static_cast<double>(k * imuPerScan + i) * IMU_PERIOD;

            // The vehicle is level: only vibration tilts the measured acceleration.
            const double phase = 2 * M_PI * VIB_FREQ * t;
            const double pitch =
                mrpt::DEG2RAD(VIB_AMPLITUDE * std::sin(phase) + rng.drawGaussian1D(0, VIB_NOISE));
            const double roll = mrpt::DEG2RAD(
                VIB_AMPLITUDE * std::cos(1.3 * phase) + rng.drawGaussian1D(0, VIB_NOISE));
            const auto tilt = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0, 0, 0, pitch, roll);
            const auto acc  = tilt.inverseComposePoint(mrpt::math::TPoint3D(0, 0, GRAVITY));

            mrpt::obs::CObservationIMU obs;
            obs.timestamp   = mrpt::Clock::fromDouble(t);
            obs.sensorLabel = "imu";
            obs.set(mrpt::obs::IMU_X_ACC, acc.x);
            obs.set(mrpt::obs::IMU_Y_ACC, acc.y);
            obs.set(mrpt::obs::IMU_Z_ACC, acc.z);
            obs.set(mrpt::obs::IMU_WX, rng.drawGaussian1D(0, GYRO_NOISE));
            obs.set(mrpt::obs::IMU_WY, rng.drawGaussian1D(0, GYRO_NOISE));
            obs.set(mrpt::obs::IMU_WZ, rng.drawGaussian1D(0, GYRO_NOISE));
            est.fuse_imu(obs);
        }

        // LiDAR pose of the scan, which arrives after the IMU readings above:
        const double                    tScan = static_cast<double>(k) * LIDAR_PERIOD;
        mrpt::poses::CPose3DPDFGaussian p;
        p.mean = mrpt::poses::CPose3D::FromXYZYawPitchRoll(SPEED * tScan, 0, 0, 0, 0, 0);
        p.cov.setIdentity();
        p.cov *= 1e-4;
        est.fuse_pose(mrpt::Clock::fromDouble(tScan), p, "lidar");

        const double tLatestImu = static_cast<double>((k + 1) * imuPerScan - 1) * IMU_PERIOD;
        const auto   st =
            est.estimated_navstate(mrpt::Clock::fromDouble(tLatestImu + PREDICT_AHEAD), "map");
        if (!st || tScan < WARMUP_DURATION)
        {
            continue;
        }
        const double tilt = mrpt::RAD2DEG(
            std::max(std::abs(st->pose.mean.pitch()), std::abs(st->pose.mean.roll())));
        maxTilt = std::max(maxTilt, tilt);
    }
    return maxTilt;
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        // Settings before averaging existed: one tight gravity factor per decimated reading.
        const double legacyTilt = run_session(0.4, 0.0);
        const double newTilt    = run_session(2.0, 1.0);

        std::cout << "Max predicted tilt: legacy " << legacyTilt << " deg, averaged " << newTilt
                  << " deg\n";

        ASSERT_LT_(newTilt, MAX_TILT_ERR);
        ASSERT_LT_(newTilt, legacyTilt);

        std::cout << "✅ SUCCESS\n";
        return 0;
    }
    catch (std::exception& e)
    {
        std::cerr << "❌ FAILED: " << e.what() << std::endl;
        return 1;
    }
}
