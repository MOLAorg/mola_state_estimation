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

namespace
{
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
    sliding_window_length: 5.0

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
void run_test()
{
    mola::state_estimation_smoother::StateEstimationSmoother stateEst;

    stateEst.initialize(mrpt::containers::yaml::FromText(navStateParams));

    const size_t numPoses = 20;

    const double T = 0.1;  // sensors period

    mrpt::poses::CPose3D actualVehiclePose = mrpt::poses::CPose3D::Identity();

    mrpt::topography::TGeodeticCoords actualVehicleInitialGeoCoords;
    actualVehicleInitialGeoCoords.lat    = 4.0;
    actualVehicleInitialGeoCoords.lon    = 3.0;
    actualVehicleInitialGeoCoords.height = 50.0;

    auto& rng = mrpt::random::getRandomGenerator();

    for (size_t i = 0; i <= numPoses; i++)
    {
        // Generate a increment in motion:
        const double t = T * static_cast<double>(i);

        // 1. Simulate Trajectory (Circular motion)
        if (i > 0)
        {
            const double v_lin = 1.0;  // m/s
            const double v_ang = 0.1;  // rad/s

            // Increment in local frame
            const auto deltaPose = mrpt::poses::CPose3D(v_lin * T, 0.0, 0.0, v_ang * T, 0.0, 0.0);

            actualVehiclePose = actualVehiclePose + deltaPose;
        }

        // 2. Simulate noisy odometry:
        mrpt::obs::CObservationOdometry obsOdo;
        obsOdo.timestamp   = mrpt::Clock::fromDouble(t);
        obsOdo.sensorLabel = "odometry";

        // Add realistic noise to odometry (accumulating drift)
        // Note: In a real scenario, noise is incremental. Here we just add noise
        // to the absolute GT to simulate a drifting input.
        auto noisyOdoPose = mrpt::poses::CPose2D(actualVehiclePose);
        noisyOdoPose.x_incr(rng.drawGaussian1D(0, 0.05));
        noisyOdoPose.y_incr(rng.drawGaussian1D(0, 0.05));
        noisyOdoPose.phi_incr(rng.drawGaussian1D(0, 0.01));

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

        // Add noise to GNSS (Approx 1.0 meter noise)
        // 1 deg Lat ~= 111km -> 1m ~= 9e-6 deg
        const double gnss_noise_deg = 1.0 / 111139.0;

        mrpt::obs::gnss::Message_NMEA_GGA gga_msg;
        gga_msg.fields.latitude_degrees =
            currentGeoCoords.lat + rng.drawGaussian1D(0, gnss_noise_deg);
        gga_msg.fields.longitude_degrees =
            currentGeoCoords.lon + rng.drawGaussian1D(0, gnss_noise_deg);
        gga_msg.fields.altitude_meters = currentGeoCoords.height + rng.drawGaussian1D(0, 1.5);
        obsGps.setMsg(gga_msg);

        // Set GNSS Covariance (in meters)
        auto& cov = obsGps.covariance_enu.emplace();
        cov.setIdentity();
        cov(0, 0) = cov(1, 1) = 1.0;  // 1m variance xy
        cov(2, 2)             = 2.25;  // 1.5m std dev z

        // Send both to state estimator:
        stateEst.fuse_odometry(obsOdo);
        stateEst.fuse_gnss(obsGps);
    }

    // Recover pose:
    const double last_t   = T * static_cast<double>(numPoses);
    const auto   stateOpt = stateEst.estimated_navstate(
        mrpt::Clock::fromDouble(last_t), stateEst.params.reference_frame_name);

    ASSERT_(stateOpt.has_value());
    std::cout << "State: " << stateOpt->asString() << "\n";
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        run_test();
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
