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
 * @file   test-imu-propagation-prediction.cpp
 * @brief  Parameters::imu_propagation: a prediction between two front-end poses
 *         follows a turn that reverses between them (a hand-held sensor
 *         swinging), measured by the gyroscope; a constant angular velocity
 *         extrapolated from the last pose cannot.
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr double T_POSE    = 0.1;  // front-end period [s]
constexpr double T_IMU     = 0.005;  // IMU period [s]
constexpr double AMPLITUDE = mrpt::DEG2RAD(8.0);  // yaw swing [rad]
constexpr double FREQ      = 2.0;  // yaw swing [Hz]
constexpr double SPEED     = 1.2;  // forward [m/s]
constexpr double DURATION  = 6.0;  // [s]

double yaw_at(double t) { return AMPLITUDE * std::sin(2 * M_PI * FREQ * t); }
double yaw_rate_at(double t) { return AMPLITUDE * 2 * M_PI * FREQ * std::cos(2 * M_PI * FREQ * t); }

mrpt::poses::CPose3D pose_at(double t)
{
    // Walking along x while the sensor swings in yaw:
    return mrpt::poses::CPose3D(SPEED * t, 0, 0, yaw_at(t), 0, 0);
}

std::string make_config(bool imuPropagation)
{
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
    imu_normalized_gravity_alignment_sigma: 0
    imu_angular_velocity_sigma: 0
    imu_propagation: )###") +
           (imuPropagation ? "true" : "false") + "\n";
}

/// Largest yaw error [rad] of the predictions at each front-end stamp, made
/// before that pose is fused, in the frame `queryFrame`.
double max_predicted_yaw_error(bool imuPropagation, const std::string& queryFrame)
{
    mola::state_estimation_smoother::StateEstimationSmoother est;
    est.initialize(mrpt::containers::yaml::FromText(make_config(imuPropagation)));

    {
        auto cov = mrpt::math::CMatrixDouble66::Identity();
        cov *= 1e-6;
        est.fuse_pose(
            mrpt::Clock::fromDouble(0),
            mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D::Identity(), cov),
            est.parameters().reference_frame_name);
    }

    double maxErr = 0;
    double tImu   = 0;
    for (double t = T_POSE; t <= DURATION; t += T_POSE)
    {
        // The IMU readings up to this front-end stamp arrive first:
        for (; tImu <= t + 1e-9; tImu += T_IMU)
        {
            mrpt::obs::CObservationIMU imu;
            imu.timestamp   = mrpt::Clock::fromDouble(tImu);
            imu.sensorLabel = "imu";
            imu.set(mrpt::obs::IMU_WX, 0.0);
            imu.set(mrpt::obs::IMU_WY, 0.0);
            imu.set(mrpt::obs::IMU_WZ, yaw_rate_at(tImu));
            est.fuse_imu(imu);
        }

        // Prediction for this stamp, after a settling second:
        if (t > 1.0)
        {
            const auto st = est.estimated_navstate(mrpt::Clock::fromDouble(t), queryFrame);
            ASSERT_(st.has_value());
            const double err =
                std::abs(mrpt::math::wrapToPi(st->pose.mean.yaw() - pose_at(t).yaw()));
            maxErr = std::max(maxErr, err);
        }

        mrpt::poses::CPose3DPDFGaussian odo;
        odo.mean = pose_at(t);
        odo.cov.setIdentity();
        odo.cov *= mrpt::square(0.01);
        est.fuse_pose(mrpt::Clock::fromDouble(t), odo, "odom");
    }
    return maxErr;
}

void test_prediction_follows_reversing_turn(const std::string& queryFrame)
{
    const double errWith    = max_predicted_yaw_error(true, queryFrame);
    const double errWithout = max_predicted_yaw_error(false, queryFrame);
    if (VERBOSE)
    {
        std::cout << "[" << queryFrame << "] max predicted yaw error: with imu_propagation "
                  << mrpt::RAD2DEG(errWith) << " deg, without " << mrpt::RAD2DEG(errWithout)
                  << " deg\n";
    }
    // The constant-twist extrapolation is degrees off at the reversals:
    ASSERT_GT_(errWithout, mrpt::DEG2RAD(2.0));
    if (queryFrame == "odom")
    {
        // Anchored on the front end's own last pose, the gyroscope follows them:
        ASSERT_LT_(errWith, mrpt::DEG2RAD(0.5));
    }
    else
    {
        // Anchored on a solved keyframe, whose own error remains:
        ASSERT_LT_(errWith, 0.6 * errWithout);
    }
}

}  // namespace

int main()
{
    try
    {
        test_prediction_follows_reversing_turn("odom");
        test_prediction_follows_reversing_turn("map");
        std::cout << "Test successful." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed with exception:\n" << e.what() << "\n";
        return 1;
    }
}
