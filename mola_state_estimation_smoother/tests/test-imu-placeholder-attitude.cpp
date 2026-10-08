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
 * @file   test-imu-placeholder-attitude.cpp
 * @brief  Unit test: imu_attitude_sigma_deg=0 ignores IMU attitude readings,
 *         e.g. a constant placeholder orientation that contradicts the
 *         accelerometer. Also covers a known geo-reference given with an
 *         all-zero ("exactly known") covariance.
 */

#include <mola_kernel/Georeferencing.h>
#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/format.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>

#include <cmath>
#include <iostream>

using namespace mrpt::literals;

namespace
{
const size_t numSteps = 100;
const double T        = 0.05;  // 20 Hz

const char* navStateParams =
    R"###(# Config for Parameters
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"

    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 5.0
    relative_pose_increment_sigma_lin: 0.02
    relative_pose_increment_sigma_ang: 0.005

    max_time_to_use_velocity_model: 2.0

    sigma_random_walk_acceleration_linear: 2.0
    sigma_random_walk_acceleration_angular: 1.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.5

    imu_normalized_gravity_alignment_sigma: 0.4
    imu_attitude_sigma_deg: %f

    link_first_pose_to_reference_origin_sigma: 0.01
)###";

/// Returns the estimated vehicle roll [rad] in the map frame.
double run(const double imuAttitudeSigmaDeg)
{
    mola::state_estimation_smoother::StateEstimationSmoother stateEst;
    stateEst.initialize(
        mrpt::containers::yaml::FromText(mrpt::format(navStateParams, imuAttitudeSigmaDeg)));

    // A known geo-reference ties {map} to ENU, so attitude readings act on the
    // vehicle pose instead of being absorbed by the T_enu_to_map estimate:
    mola::Georeferencing georef;
    georef.geo_coord.lat     = 36.8;
    georef.geo_coord.lon     = -2.4;
    georef.T_enu_to_map.mean = mrpt::poses::CPose3D::Identity();
    georef.T_enu_to_map.cov.setZero();  // "exactly known"
    stateEst.set_geo_reference(georef);

    // The placeholder: an orientation claiming a 90 deg roll, while the vehicle
    // is level, as the accelerometer correctly reports.
    const auto bogusAttitude = mrpt::poses::CPose3D::FromYawPitchRoll(0, 0, 90.0_deg);
    mrpt::math::CQuaternionDouble q;
    bogusAttitude.getAsQuaternion(q);

    mrpt::poses::CPose3D pose;
    for (size_t i = 1; i <= numSteps; i++)
    {
        const auto time = mrpt::Clock::fromDouble(T * static_cast<double>(i));

        pose = pose + mrpt::poses::CPose3D(0.5 * T, 0, 0, 0.1 * T, 0, 0);

        mrpt::poses::CPose3DPDFGaussian posePdf;
        posePdf.mean = pose;
        posePdf.cov.setIdentity();
        posePdf.cov *= 1e-3;

        mrpt::obs::CObservationIMU obsImu;
        obsImu.timestamp   = time;
        obsImu.sensorLabel = "imu";
        obsImu.set(mrpt::obs::IMU_X_ACC, 0.0);
        obsImu.set(mrpt::obs::IMU_Y_ACC, 0.0);
        obsImu.set(mrpt::obs::IMU_Z_ACC, 9.81);
        obsImu.set(mrpt::obs::IMU_ORI_QUAT_W, q.r());
        obsImu.set(mrpt::obs::IMU_ORI_QUAT_X, q.x());
        obsImu.set(mrpt::obs::IMU_ORI_QUAT_Y, q.y());
        obsImu.set(mrpt::obs::IMU_ORI_QUAT_Z, q.z());

        stateEst.fuse_pose(time, posePdf, "lidar");
        stateEst.fuse_imu(obsImu);
    }

    const auto state = stateEst.estimated_navstate(mrpt::Clock::fromDouble(T * numSteps), "map");
    ASSERT_(state.has_value());

    const double roll = state->pose.mean.roll();
    std::cout << "imu_attitude_sigma_deg=" << imuAttitudeSigmaDeg
              << " => roll in map: " << mrpt::RAD2DEG(roll) << " deg\n";
    return roll;
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        // Fused, the placeholder tilts the level vehicle by a few degrees:
        const double rollWithAttitude = run(2.0);
        ASSERT_(std::abs(rollWithAttitude) > mrpt::DEG2RAD(1.0));

        // Ignored, the accelerometer alone keeps it level:
        const double rollIgnored = run(0.0);
        ASSERT_NEAR_(rollIgnored, 0.0, mrpt::DEG2RAD(0.1));
        std::cout << "✅ SUCCESS\n";
        return 0;
    }
    catch (std::exception& e)
    {
        std::cerr << "❌ FAILED: " << e.what() << std::endl;
        return 1;
    }
}
