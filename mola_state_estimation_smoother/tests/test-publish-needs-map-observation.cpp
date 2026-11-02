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
 * @file   test-publish-needs-map-observation.cpp
 * @brief  With a fixed geo-reference, spinOnce() must not publish a {map}
 *         pose before some source observes {map}: until then it is only the
 *         first odometry frame, and the pose would be the raw odometry one.
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace
{
constexpr double POSE_DT   = 0.05;
constexpr size_t NUM_POSES = 20;

// The odometry frame origin is far from the map one, as with a robot whose
// odometry started elsewhere:
constexpr double ODOM_X0 = -64.8;
constexpr double ODOM_Y0 = 9.97;

std::string params_yaml(bool async)
{
    return std::string(R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 10.0
    relative_pose_increment_sigma_lin: 0.02
    relative_pose_increment_sigma_ang: 0.005
    max_time_to_use_velocity_model: 5.0
    sigma_random_walk_acceleration_linear: 2.0
    sigma_random_walk_acceleration_angular: 1.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.10
    estimate_geo_reference: false
    imu_attitude_sigma_deg: 3.0
    async_backend: )###") +
           (async ? "true" : "false") + "\n";
}

using LU = mola::LocalizationSourceBase::LocalizationUpdate;

size_t count_vehicle_updates(const std::vector<LU>& v)
{
    size_t n = 0;
    for (const auto& l : v)
    {
        if (l.child_frame == "base_link" && l.reference_frame == "map")
        {
            n++;
        }
    }
    return n;
}

mrpt::poses::CPose3DPDFGaussian pose(double x, double y, double sigma)
{
    mrpt::poses::CPose3DPDFGaussian p;
    p.mean = mrpt::poses::CPose3D(x, y, 0, 0, 0, 0);
    p.cov.setDiagonal(sigma * sigma);
    return p;
}

// Spins until a {map} vehicle update is advertised, or about 4 s.
void spin_until_published(
    mola::state_estimation_smoother::StateEstimationSmoother& est, const std::vector<LU>& got)
{
    for (int i = 0; i < 800 && count_vehicle_updates(got) == 0; i++)
    {
        est.spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

// A level vehicle, as seen by an IMU with absolute attitude (AHRS):
mrpt::obs::CObservationIMU level_attitude(const mrpt::Clock::time_point& t)
{
    mrpt::obs::CObservationIMU imu;
    imu.timestamp   = t;
    imu.sensorLabel = "imu";
    imu.set(mrpt::obs::IMU_X_ACC, 0.0);
    imu.set(mrpt::obs::IMU_Y_ACC, 0.0);
    imu.set(mrpt::obs::IMU_Z_ACC, 9.81);
    imu.set(mrpt::obs::IMU_ORI_QUAT_W, 1.0);
    imu.set(mrpt::obs::IMU_ORI_QUAT_X, 0.0);
    imu.set(mrpt::obs::IMU_ORI_QUAT_Y, 0.0);
    imu.set(mrpt::obs::IMU_ORI_QUAT_Z, 0.0);
    return imu;
}

// withAttitude: an absolute attitude observes the orientation of {map}, but
// not its position, so it must not enable the {map} output by itself.
void run_test(bool async, bool withGeoref, bool withAttitude)
{
    mola::state_estimation_smoother::StateEstimationSmoother est;
    if (mrpt::get_env<bool>("VERBOSE", false))
    {
        est.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    est.initialize(mrpt::containers::yaml::FromText(params_yaml(async)));

    if (withGeoref)
    {
        mola::Georeferencing georef;
        georef.geo_coord.lat     = 36.83;
        georef.geo_coord.lon     = -2.41;
        georef.geo_coord.height  = 50.0;
        georef.T_enu_to_map.mean = mrpt::poses::CPose3D::Identity();
        georef.T_enu_to_map.cov.setZero();
        est.set_geo_reference(georef);
    }

    std::vector<LU> got;
    est.subscribeToLocalizationUpdates([&got](const LU& l) { got.push_back(l); });

    // Odometry only (a still vehicle):
    mrpt::Clock::time_point last;
    for (size_t i = 0; i < NUM_POSES; i++)
    {
        last = mrpt::Clock::fromDouble(1000.0 + POSE_DT * static_cast<double>(i));
        est.fuse_pose(last, pose(ODOM_X0, ODOM_Y0, 0.01), "odom");
        if (withAttitude)
        {
            est.fuse_imu(level_attitude(last));
        }
    }
    for (int i = 0; i < 100; i++)
    {
        est.spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    const size_t beforeMap = count_vehicle_updates(got);
    std::cout << "async=" << async << " georef=" << withGeoref << " attitude=" << withAttitude
              << ": {map} updates before observing it: " << beforeMap << "\n";

    if (!withGeoref)
    {
        // Odometry-only fusion keeps publishing with {map} = first odometry frame:
        spin_until_published(est, got);
        ASSERT_GT_(count_vehicle_updates(got), 0U);
        return;
    }
    ASSERT_EQUAL_(beforeMap, 0U);

    // Now a source observes {map} (e.g. a localization from a front end):
    est.fuse_pose(last + std::chrono::milliseconds(10), pose(1.0, -3.5, 0.05), "map");
    spin_until_published(est, got);
    std::cout << "  {map} updates after observing it: " << count_vehicle_updates(got) << "\n";
    ASSERT_GT_(count_vehicle_updates(got), 0U);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        for (const bool async : {false, true})
        {
            run_test(async, true, false);
            run_test(async, true, true);
            run_test(async, false, false);
        }
        std::cout << "Test successful.\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
