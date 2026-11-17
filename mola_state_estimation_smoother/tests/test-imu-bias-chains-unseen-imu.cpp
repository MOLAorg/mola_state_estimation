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
 * @file   test-imu-bias-chains-unseen-imu.cpp
 * @brief  IMU bias variables exist only for the IMUs actually seen: with fewer
 *         IMUs than imu_preintegration.max_imus, or none at all, every solve
 *         must succeed.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose2D.h>

#include <cmath>
#include <iostream>
#include <string>

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr double IMU_DT     = 0.005;  // [s]  (200 Hz)
constexpr double ODOM_DT    = 0.1;  // [s]  (10 Hz)
constexpr double DURATION   = 6.0;  // [s]
constexpr double VELOCITY_X = 1.0;  // [m/s]
constexpr double YAW_RATE   = 0.2;  // [rad/s]

std::string params_yaml(bool gyroOnly, int maxImus)
{
    return R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    link_first_pose_to_reference_origin_sigma: 1e-6
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 10.0
    relative_pose_increment_sigma_lin: 0.02
    relative_pose_increment_sigma_ang: 0.005
    max_time_to_use_velocity_model: 2.0
    sigma_random_walk_acceleration_linear: 2.0
    sigma_random_walk_acceleration_angular: 1.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.10
    estimate_geo_reference: false
    imu_min_sample_period: 0.1
    imu_preintegration:
        enabled: true
        gyro_only: )###" +
           std::string(gyroOnly ? "true" : "false") +
           "\n        max_imus: " + std::to_string(maxImus) + "\n";
}

/// Wheel odometry along a circle, plus `numImus` (0 or 1) IMUs.
void run_case(bool gyroOnly, int maxImus, int numImus)
{
    std::cout << "gyro_only: " << gyroOnly << ", max_imus: " << maxImus
              << ", IMUs present: " << numImus << "\n";

    mola::state_estimation_smoother::StateEstimationSmoother est;
    if (VERBOSE)
    {
        est.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    est.initialize(mrpt::containers::yaml::FromText(params_yaml(gyroOnly, maxImus)));

    const auto   numReadings  = static_cast<size_t>(DURATION / IMU_DT) + 1;
    const size_t imuPerOdom   = static_cast<size_t>(std::lround(ODOM_DT / IMU_DT));
    size_t       numEstimates = 0;
    for (size_t i = 0; i < numReadings; i++)
    {
        const double t     = IMU_DT * static_cast<double>(i);
        const auto   stamp = mrpt::Clock::fromDouble(t);
        const double yaw   = YAW_RATE * t;
        const double r     = VELOCITY_X / YAW_RATE;

        if (numImus > 0)
        {
            mrpt::obs::CObservationIMU imu;
            imu.timestamp   = stamp;
            imu.sensorLabel = "imu";
            // Centripetal acceleration, in the vehicle frame:
            imu.set(mrpt::obs::IMU_X_ACC, 0.0);
            imu.set(mrpt::obs::IMU_Y_ACC, VELOCITY_X * YAW_RATE);
            imu.set(mrpt::obs::IMU_Z_ACC, 9.81);
            imu.set(mrpt::obs::IMU_WX, 0.0);
            imu.set(mrpt::obs::IMU_WY, 0.0);
            imu.set(mrpt::obs::IMU_WZ, YAW_RATE);
            est.fuse_imu(imu);
        }

        if (i % imuPerOdom != 0)
        {
            continue;
        }

        mrpt::obs::CObservationOdometry odom;
        odom.timestamp = stamp;
        odom.odometry  = mrpt::poses::CPose2D(r * std::sin(yaw), r * (1 - std::cos(yaw)), yaw);
        est.fuse_odometry(odom, "wheels_odom");

        // Every solve must succeed (a failed one resets the estimator):
        if (t > 0.5)
        {
            numEstimates++;
            const auto nav = est.estimated_navstate(stamp, "map");
            ASSERT_(nav.has_value());
            const double err = std::hypot(
                nav->pose.mean.x() - odom.odometry.x(), nav->pose.mean.y() - odom.odometry.y());
            ASSERT_LT_(err, 0.5);  // a reset would land meters away
        }
    }
    ASSERT_GT_(numEstimates, 50U);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        run_case(false, 1, 0);
        run_case(false, 1, 1);
        run_case(true, 1, 0);
        run_case(true, 1, 1);
        run_case(true, 3, 1);
        std::cout << "✅ SUCCESS\n";
        return 0;
    }
    catch (std::exception& e)
    {
        std::cerr << "❌ FAILED: " << e.what() << std::endl;
        return 1;
    }
}
