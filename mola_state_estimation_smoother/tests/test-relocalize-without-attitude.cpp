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
 * @file   test-relocalize-without-attitude.cpp
 * @brief  In relocalize mode with GNSS and wheel odometry but no attitude
 *         source, a static vehicle's orientation is not observed: the
 *         estimator must still solve, report the position, and report the
 *         orientation as unknown until some source (here, a pose in the map
 *         frame) observes it.
 */

#include <mola_kernel/interfaces/NavStateFilter.h>
#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/topography/conversions.h>

#include <cmath>
#include <iostream>
#include <tuple>

#if defined(MOLA_KERNEL_NAVSTATE_FILTER_HAS_GEO_REFERENCE)

namespace
{
const char* navStateParams =
    R"###(# Config for Parameters
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    max_time_to_use_velocity_model: 2.0
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 8.0
    relative_pose_increment_sigma_lin: 0.02
    relative_pose_increment_sigma_ang: 0.005
    min_time_difference_to_create_new_frame: 0.05
    sigma_random_walk_acceleration_linear: 0.1
    sigma_random_walk_acceleration_angular: 0.1
    sigma_integrator_position: 0.02
    sigma_integrator_orientation: 0.02
    estimate_geo_reference: false
    async_backend: false
    convergence_max_position_sigma: 1.0
    convergence_max_orientation_sigma_deg: 360.0
)###";

constexpr double kVehicleX = 12.0;
constexpr double kVehicleY = -5.0;

mrpt::obs::CObservationGPS gnss(const mola::Georeferencing& georef, double t)
{
    mrpt::math::TPoint3D geocentric;
    mrpt::topography::ENUToGeocentric(
        {kVehicleX, kVehicleY, 0}, georef.geo_coord, geocentric,
        mrpt::topography::TEllipsoid::Ellipsoid_WGS84());
    mrpt::topography::TGeodeticCoords geo;
    mrpt::topography::geocentricToGeodetic(geocentric, geo);

    mrpt::obs::gnss::Message_NMEA_GGA gga;
    gga.fields.latitude_degrees  = geo.lat.decimal_value;
    gga.fields.longitude_degrees = geo.lon.decimal_value;
    gga.fields.altitude_meters   = geo.height;
    gga.fields.fix_quality       = 1;

    mrpt::obs::CObservationGPS obs;
    obs.sensorLabel = "gps";
    obs.timestamp   = mrpt::Clock::fromDouble(t);
    obs.setMsg(gga);
    auto& cov = obs.covariance_enu.emplace();
    cov.setDiagonal(std::vector<double>({0.25, 0.25, 1.0}));
    return obs;
}

double yaw_sigma_deg(const mrpt::poses::CPose3DPDFGaussian& p)
{
    return mrpt::RAD2DEG(std::sqrt(p.cov(3, 3)));
}

void run_test()
{
    mola::state_estimation_smoother::StateEstimationSmoother stateEst;
    if (mrpt::get_env<bool>("VERBOSE", false))
    {
        stateEst.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    stateEst.initialize(mrpt::containers::yaml::FromText(navStateParams));

    mola::Georeferencing georef;
    georef.geo_coord.lat     = 36.87;
    georef.geo_coord.lon     = -2.34;
    georef.geo_coord.height  = 100.0;
    georef.T_enu_to_map.mean = mrpt::poses::CPose3D::Identity();
    georef.T_enu_to_map.cov.setZero();
    stateEst.set_geo_reference(georef);

    // Static vehicle: wheel odometry and GNSS, no attitude source.
    mrpt::poses::CPose3DPDFGaussian odo;
    odo.cov.setDiagonal(std::vector<double>({0.01, 0.01, 0.01, 1e-4, 1e-4, 1e-4}));

    double                          t = 0;
    mrpt::poses::CPose3DPDFGaussian pose;
    bool                            converged = false;
    for (int i = 0; i < 50; i++, t += 0.1)
    {
        stateEst.fuse_pose(mrpt::Clock::fromDouble(t), odo, "odom_wheels");
        if (i % 10 == 0)
        {
            stateEst.fuse_gnss(gnss(georef, t));
        }
        std::ignore = stateEst.estimated_navstate(mrpt::Clock::fromDouble(t), "map");
        converged   = stateEst.has_converged_localization(pose);
    }
    if (!converged)
    {
        throw std::runtime_error("Did not converge on position with GNSS");
    }
    std::cout << "Converged at " << pose.mean.asString() << " yaw sigma " << yaw_sigma_deg(pose)
              << " deg\n";
    if (std::hypot(pose.mean.x() - kVehicleX, pose.mean.y() - kVehicleY) > 0.5)
    {
        throw std::runtime_error("Converged far from the GNSS position");
    }
    if (yaw_sigma_deg(pose) < 90.0)
    {
        throw std::runtime_error("The unobserved yaw was reported as known");
    }

    // A pose in the map frame observes the orientation:
    mrpt::poses::CPose3DPDFGaussian inMap;
    inMap.mean = mrpt::poses::CPose3D::FromXYZYawPitchRoll(kVehicleX, kVehicleY, 0, 1.0, 0, 0);
    inMap.cov.setDiagonal(std::vector<double>({1e-2, 1e-2, 1e-2, 1e-4, 1e-4, 1e-4}));
    for (int i = 0; i < 10; i++, t += 0.1)
    {
        stateEst.fuse_pose(mrpt::Clock::fromDouble(t), odo, "odom_wheels");
        stateEst.fuse_pose(mrpt::Clock::fromDouble(t), inMap, "map");
        std::ignore = stateEst.estimated_navstate(mrpt::Clock::fromDouble(t), "map");
    }
    if (!stateEst.has_converged_localization(pose))
    {
        throw std::runtime_error("Not converged with map poses");
    }
    std::cout << "With map poses: " << pose.mean.asString() << " yaw sigma " << yaw_sigma_deg(pose)
              << " deg\n";
    if (yaw_sigma_deg(pose) > 10.0 || std::abs(pose.mean.yaw() - 1.0) > 0.05)
    {
        throw std::runtime_error("Orientation not taken from the map poses");
    }
}
}  // namespace

int main()
{
    try
    {
        run_test();
        std::cout << "\nSUCCESS\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "\nFAILED:\n" << e.what() << std::endl;
        return 1;
    }
}

#else

int main()
{
    std::cout << "\nSKIPPED: installed mola_kernel lacks "
                 "NavStateFilter::set_geo_reference()/get_geo_reference()\n";
    return 0;
}
#endif
