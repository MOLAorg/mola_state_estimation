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
 * @file   test-gauge-anchors.cpp
 * @brief  Unit tests for the directions no measurement observes (gauge
 *         freedoms): where {map} is while only odometry-frame sources exist, and
 *         the azimuth of T_enu_to_map with gravity-only IMU readings. The solver
 *         must neither fail nor drift in them, and must release {map} as soon as
 *         a source reports poses in it.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/random/RandomGenerators.h>

#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <string>

using namespace mrpt::literals;

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr double IMU_PERIOD  = 0.01;  // [s]
constexpr double POSE_PERIOD = 0.1;  // [s]
constexpr double DURATION    = 12.0;  // [s]
constexpr double POSE_NOISE  = 0.005;  // [m], [rad]

// Deliberately without link_first_pose_to_reference_origin_sigma: nothing but
// the sources themselves may define {map}.
const char* navStateParams = R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 5.0
    max_time_to_use_velocity_model: 2.0
    sigma_random_walk_acceleration_linear: 1.0
    sigma_random_walk_acceleration_angular: 1.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.10
    estimate_geo_reference: false
    async_backend: false
)###";

// Ground truth in {map}: constant forward speed plus a gentle turn.
mrpt::poses::CPose3D gtPoseAt(double t)
{
    const double yaw = 0.2 * t;
    const double r   = 1.0 / 0.2;
    return mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        r * std::sin(yaw), r * (1.0 - std::cos(yaw)), 0, yaw, 0, 0);
}

struct Session
{
    size_t                   solverFailures = 0;
    std::map<double, double> errorAt;  //!< time -> position error [m]
    std::optional<double>    firstEstimateTime;
};

/** Streams IMU readings from t=0, plus poses of a source whose frame sits at
 *  `T_map_to_odom`, and, from `mapPosesFrom` on, poses directly in {map}.
 *  `expected(t)` is the pose the estimate must report in {map} at time t. */
Session run(
    const mrpt::poses::CPose3D& T_map_to_odom, std::optional<double> mapPosesFrom,
    const std::function<mrpt::poses::CPose3D(double)>& expected)
{
    auto& rng = mrpt::random::getRandomGenerator();
    rng.randomize(1234);

    mola::state_estimation_smoother::StateEstimationSmoother est;
    if (VERBOSE)
    {
        est.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    est.initialize(mrpt::containers::yaml::FromText(navStateParams));

    Session res;
    est.logRegisterCallback(
        [&res](
            std::string_view msg, const mrpt::system::VerbosityLevel level, std::string_view,
            const mrpt::Clock::time_point)
        {
            if (level >= mrpt::system::LVL_ERROR &&
                msg.find("GTSAM update failed") != std::string_view::npos)
            {
                res.solverFailures++;
            }
        });

    const auto noisyPdf = [&](const mrpt::poses::CPose3D& p)
    {
        mrpt::poses::CPose3DPDFGaussian pdf;
        pdf.mean = p + mrpt::poses::CPose3D::FromXYZYawPitchRoll(
                           rng.drawGaussian1D(0, POSE_NOISE), rng.drawGaussian1D(0, POSE_NOISE),
                           rng.drawGaussian1D(0, POSE_NOISE), rng.drawGaussian1D(0, POSE_NOISE),
                           rng.drawGaussian1D(0, POSE_NOISE), rng.drawGaussian1D(0, POSE_NOISE));
        pdf.cov.setIdentity();
        pdf.cov *= mrpt::square(POSE_NOISE);
        return pdf;
    };

    const size_t nPoseSteps = static_cast<size_t>(POSE_PERIOD / IMU_PERIOD + 0.5);

    for (size_t i = 0; IMU_PERIOD * static_cast<double>(i) <= DURATION; i++)
    {
        const double t     = IMU_PERIOD * static_cast<double>(i);
        const auto   stamp = mrpt::Clock::fromDouble(t);

        // Gravity-only IMU (the map is level here), readings ahead of any pose:
        mrpt::obs::CObservationIMU imu;
        imu.timestamp   = stamp;
        imu.sensorLabel = "imu";
        imu.set(mrpt::obs::IMU_X_ACC, rng.drawGaussian1D(0, 0.05));
        imu.set(mrpt::obs::IMU_Y_ACC, rng.drawGaussian1D(0, 0.05));
        imu.set(mrpt::obs::IMU_Z_ACC, 9.81 + rng.drawGaussian1D(0, 0.05));
        est.fuse_imu(imu);

        if (i % nPoseSteps != 0)
        {
            continue;
        }

        // Queried before the first pose too: must be "not ready yet", not a failure.
        if (i == 0)
        {
            ASSERT_(!est.estimated_navstate(stamp, "map").has_value());
            continue;
        }

        const auto gt = gtPoseAt(t);
        est.fuse_pose(stamp, noisyPdf(-T_map_to_odom + gt), "odom");
        if (mapPosesFrom && t >= *mapPosesFrom)
        {
            est.fuse_pose(stamp, noisyPdf(gt), "map");
        }

        if (const auto st = est.estimated_navstate(stamp, "map"); st.has_value())
        {
            if (!res.firstEstimateTime)
            {
                res.firstEstimateTime = t;
            }
            res.errorAt[t] = (st->pose.mean.translation() - expected(t).translation()).norm();
            if (VERBOSE)
            {
                std::cout << "t=" << t << " est=" << st->pose.mean.asString()
                          << " expected=" << expected(t).asString() << "\n";
            }
        }
    }
    return res;
}

// Only odometry-frame poses: {map} is defined as that first odometry frame, even
// when the source starts far from its own origin.
void test_odometry_frame_defines_map()
{
    const auto T_map_to_odom =
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(-20.0, 10.0, 0.5, 42.0_deg, 0, 0);

    // {map} == {odom}, so the estimate follows the source's own coordinates:
    const auto res =
        run(T_map_to_odom, std::nullopt, [&](double t) { return -T_map_to_odom + gtPoseAt(t); });

    ASSERT_EQUAL_(res.solverFailures, 0U);
    ASSERT_(res.firstEstimateTime.has_value());
    ASSERT_LT_(*res.firstEstimateTime, 0.5);

    double maxErr = 0;
    for (const auto& [t, err] : res.errorAt)
    {
        maxErr = std::max(maxErr, err);
    }
    std::cout << "[odometry_frame_defines_map] max position error: " << maxErr << " m\n";
    ASSERT_LT_(maxErr, 0.05);
}

// Poses in {map} arrive only after the odometry source has been running for a
// while: {map} must then follow them, not stay pinned to the odometry frame.
void test_late_map_poses_take_over()
{
    constexpr double MAP_POSES_FROM = 4.0;

    const auto T_map_to_odom =
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(3.0, -2.0, 0.0, 15.0_deg, 0, 0);

    const auto res = run(T_map_to_odom, MAP_POSES_FROM, [](double t) { return gtPoseAt(t); });

    ASSERT_EQUAL_(res.solverFailures, 0U);

    // Allow one window for the old {map} definition to leave the smoother:
    double maxErr   = 0;
    size_t nLateEst = 0;
    for (const auto& [t, err] : res.errorAt)
    {
        if (t > MAP_POSES_FROM + 5.0)
        {
            maxErr = std::max(maxErr, err);
            nLateEst++;
        }
    }
    ASSERT_GT_(nLateEst, 0U);
    std::cout << "[late_map_poses_take_over] max position error: " << maxErr << " m\n";
    ASSERT_LT_(maxErr, 0.05);
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_odometry_frame_defines_map();
        test_late_map_poses_take_over();

        std::cout << "All tests passed." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
