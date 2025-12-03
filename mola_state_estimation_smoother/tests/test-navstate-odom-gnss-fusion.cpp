/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this odometry package
 alone or in combination with the complete SLAM system.
*/

/**
 * @file   test-navstate-odom-gnss-fusion.cpp
 * @brief  Unit tests for StateEstimationSmoother
 * @author Jose Luis Blanco Claraco
 * @date   Nov 29, 2025
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/topography/conversions.h>

#include <iostream>

using namespace std::string_literals;
using namespace mrpt::literals;

namespace
{

constexpr double GNSS_NOISE_XY_M    = 0.10;
constexpr double GNSS_NOISE_Z_M     = 0.10;
constexpr double MOTION_LIN_VX      = 1.0;  // m/s
constexpr double MOTION_ANG_WZ      = 0.1;  // rad/s
constexpr double ODOMETRY_NOISE_XY  = 0.01;
constexpr double ODOMETRY_NOISE_PHI = 0.1_deg;

constexpr const char* ODOMETRY_NAME = "odom";

const char* navStateParams =
    R"###(# Config for Parameters
params:
    # Frame name for the vehicle/robot base
    vehicle_frame_name: "base_link"

    # Reference frame for pose publication (typically 'map' or 'odom')
    reference_frame_name: "map"

    max_time_to_use_velocity_model: 2.0

    # ----------------------------------------------------------------------------
    # Kinematic Model & Motion Factors
    # ----------------------------------------------------------------------------

    # Kinematic model for internal motion model factors
    # Options: KinematicModel::ConstantVelocity, KinematicModel::Tricycle
    kinematic_model: KinematicModel::ConstantVelocity

    # Time window to keep past observations in the filter [seconds]
    sliding_window_length: 2.0
    
    # Minimum time difference between frames to create a new frame [seconds]
    min_time_difference_to_create_new_frame: 0.01

    # Random walk model for linear acceleration uncertainty [m/s²]
    sigma_random_walk_acceleration_linear: 1.0

    # Random walk model for angular acceleration uncertainty [rad/s²]
    sigma_random_walk_acceleration_angular: 1.0

    # Integrator uncertainty for position [m]
    sigma_integrator_position: 0.10

    # Integrator uncertainty for orientation [rad]
    sigma_integrator_orientation: 0.10

    # Enable estimation of geo-referencing from GNSS and other sensors
    # If false, geo-reference must be provided externally or via fixed_geo_reference
    estimate_geo_reference: true

    # Fixed geo-reference to use when estimate_geo_reference is false
    #fixed_geo_reference: { latitude_deg: 0.0, longitude_deg: 0.0, altitude: 0.0 }
)###";

// Test: simulate a random trajectory on XY on a given latitude/longitude with arbitrary initial
// heading, then generate noisy local odometry measurements, and noisy GNSS measurements and recover
// the geo-referenced trajectory from them.
//
void run_test(const mrpt::poses::CPose3D& actualInitialPoseWrtMap)
{
    mola::state_estimation_smoother::StateEstimationSmoother stateEst;

    stateEst.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    stateEst.profiler_.enable();

    stateEst.initialize(mrpt::containers::yaml::FromText(navStateParams));

    const size_t numPoses = 40;

    const double T = 0.1;  // sensors period

    mrpt::poses::CPose3D actualVehiclePose = actualInitialPoseWrtMap;  // wrt "map" frame
    mrpt::poses::CPose2D odometryPose      = mrpt::poses::CPose2D::Identity();  // wrt "odom" frame

    mrpt::topography::TGeodeticCoords actualVehicleInitialGeoCoords;
    actualVehicleInitialGeoCoords.lat    = 4.0;
    actualVehicleInitialGeoCoords.lon    = 3.0;
    actualVehicleInitialGeoCoords.height = 50.0;

    // Link the "map" frame origin with "odom" for this toy example.
    // Otherwise, "odom" would "float" around without any particular XYZ known displacement.
    {
        auto map2odom_cov = mrpt::math::CMatrixDouble66::Identity();
        map2odom_cov *= 1e-6;

        stateEst.fuse_pose(
            mrpt::Clock::fromDouble(0),
            mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D::Identity(), map2odom_cov),
            stateEst.params.reference_frame_name);
    }

    auto& rng = mrpt::random::getRandomGenerator();
    rng.randomize(1234);

    for (size_t i = 0; i <= numPoses; i++)
    {
        // Generate a increment in motion:
        const double t = T * static_cast<double>(i);

        // 1. Simulate Trajectory (Circular motion)
        if (i > 0)
        {
            // Increment in local frame

            auto deltaPose2D     = mrpt::poses::CPose2D(MOTION_LIN_VX * T, 0.0, MOTION_ANG_WZ * T);
            const auto deltaPose = mrpt::poses::CPose3D(deltaPose2D);
            actualVehiclePose    = actualVehiclePose + deltaPose;

            deltaPose2D.x_incr(rng.drawGaussian1D(0, ODOMETRY_NOISE_XY));
            deltaPose2D.y_incr(rng.drawGaussian1D(0, ODOMETRY_NOISE_XY));
            deltaPose2D.phi_incr(rng.drawGaussian1D(0, ODOMETRY_NOISE_PHI));

            odometryPose = odometryPose + deltaPose2D;
        }

        // 2. Simulate noisy odometry:
        mrpt::obs::CObservationOdometry obsOdo;
        obsOdo.timestamp   = mrpt::Clock::fromDouble(t);
        obsOdo.sensorLabel = ODOMETRY_NAME;  // actually unused

        // Add realistic noise to odometry (accumulating drift)
        // Note: In a real scenario, noise is incremental. Here we just add noise
        // to the absolute GT to simulate a drifting input.
        auto noisyOdoPose = odometryPose;

        obsOdo.odometry        = noisyOdoPose;
        obsOdo.hasEncodersInfo = false;
        obsOdo.hasVelocities   = false;

        // 3. Simulate noisy GNSS:
        mrpt::obs::CObservationGPS obsGps;
        obsGps.timestamp   = mrpt::Clock::fromDouble(t);
        obsGps.sensorLabel = "gnss";

        // Convert current Local ENU pose to Geodetic (Lat/Lon/Alt)
        mrpt::topography::TGeocentricCoords currentGeocentricCoords;
        mrpt::topography::ENUToGeocentric(
            {actualVehiclePose.x(), actualVehiclePose.y(), actualVehiclePose.z()},
            actualVehicleInitialGeoCoords, currentGeocentricCoords,
            mrpt::topography::TEllipsoid::Ellipsoid_WGS84());

        mrpt::topography::TGeodeticCoords currentGeoCoords;
        mrpt::topography::geocentricToGeodetic(currentGeocentricCoords, currentGeoCoords);

        // sanity/validation check for geodetic -> ENU conversion:
        {
            mrpt::math::TPoint3D out_ENU_point;
            mrpt::topography::geodeticToENU_WGS84(
                currentGeoCoords, out_ENU_point, actualVehicleInitialGeoCoords);
            ASSERT_NEAR_(out_ENU_point.x, actualVehiclePose.x(), 1e-2);
            ASSERT_NEAR_(out_ENU_point.y, actualVehiclePose.y(), 1e-2);
            ASSERT_NEAR_(out_ENU_point.z, actualVehiclePose.z(), 1e-2);
        }

        // Add noise to GNSS:
        constexpr double gnss_noise_deg = GNSS_NOISE_XY_M * mrpt::RAD2DEG(1.0 / 6300e3);

        mrpt::obs::gnss::Message_NMEA_GGA gga_msg;
        gga_msg.fields.latitude_degrees =
            currentGeoCoords.lat + rng.drawGaussian1D(0, gnss_noise_deg);
        gga_msg.fields.longitude_degrees =
            currentGeoCoords.lon + rng.drawGaussian1D(0, gnss_noise_deg);
        gga_msg.fields.altitude_meters =
            currentGeoCoords.height + rng.drawGaussian1D(0, GNSS_NOISE_Z_M);
        gga_msg.fields.fix_quality = 1;
        obsGps.setMsg(gga_msg);

        // Set GNSS Covariance (in meters)
        auto& cov = obsGps.covariance_enu.emplace();
        cov.setIdentity();
        cov(0, 0) = cov(1, 1) = mrpt::square(GNSS_NOISE_XY_M);
        cov(2, 2)             = mrpt::square(GNSS_NOISE_Z_M);

        // Send both to state estimator:
        stateEst.fuse_odometry(obsOdo, ODOMETRY_NAME);
        stateEst.fuse_gnss(obsGps);

        if (i % 5 == 0)
        {
            // Enforce updating estimation:
            const auto stateOpt = stateEst.estimated_navstate(
                mrpt::Clock::fromDouble(t), stateEst.params.reference_frame_name);
            if (stateOpt)
            {
                std::cout << stateOpt->asString() << "\nGT: " << actualVehiclePose << "\n\n";
            }
        }
    }

    // Recover pose, in the reference frame:
    const double last_t = T * static_cast<double>(numPoses);
    {
        const auto stateOpt = stateEst.estimated_navstate(
            mrpt::Clock::fromDouble(last_t), stateEst.params.reference_frame_name);

        ASSERT_(stateOpt.has_value());
        std::cout << "State (ref.frame): " << stateOpt->asString() << "\n";

        // wrt map:
        const auto estimatedPoseWrtMap = stateOpt->pose.mean;

        const double final_se3_error =
            mrpt::poses::Lie::SE<3>::log(
                estimatedPoseWrtMap - (actualVehiclePose - actualInitialPoseWrtMap))
                .norm();
        std::cout << "final_se3_error: " << final_se3_error << "\n";
        ASSERT_LT_(final_se3_error, 0.30);
    }

    // Recover pose, in the odometry frame:
    ASSERT_EQUAL_(stateEst.known_odometry_frame_ids().size(), 1);

    {
        const auto stateOpt =
            stateEst.estimated_navstate(mrpt::Clock::fromDouble(last_t), ODOMETRY_NAME);

        ASSERT_(stateOpt.has_value());
        std::cout << "State (odom frame): " << stateOpt->asString() << "\n";

        const double final_se3_error =
            mrpt::poses::Lie::SE<3>::log(
                stateOpt->pose.mean - (actualVehiclePose - actualInitialPoseWrtMap))
                .norm();
        std::cout << "final_se3_error: " << final_se3_error << "\n";
        ASSERT_LT_(final_se3_error, 0.30);
    }
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        // Run for different initial poses:
        run_test(mrpt::poses::CPose3D::Identity());
        run_test(mrpt::poses::CPose3D::FromTranslation(3.0, 2.0, 1.0));
        run_test(
            mrpt::poses::CPose3D::FromXYZYawPitchRoll(1.0, 3.0, 0.5, 30.0_deg, 0.0_deg, 0.0_deg));
    }
    catch (std::exception& e)
    {
        mrpt::system::consoleColorAndStyle(mrpt::system::ConsoleForegroundColor::RED);
        std::cerr << " ERROR: " << std::endl << e.what() << std::endl;
        mrpt::system::consoleColorAndStyle(mrpt::system::ConsoleForegroundColor::DEFAULT);
        return 1;
    }

    return 0;
}
