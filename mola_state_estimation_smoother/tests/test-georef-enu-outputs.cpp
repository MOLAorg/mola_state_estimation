/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this package
 alone or in combination with the complete SLAM system.
*/

/**
 * @file   test-georef-enu-outputs.cpp
 * @brief  Live geo-referencing with odometry-frame poses and GNSS fixes stamped
 *         between keyframes, on a fast vehicle. Checks the outputs in the ENU
 *         frame: the causal navstate, the smoothed trajectory (including the
 *         keyframes finalized in the very solve where the geo-reference
 *         appears, since the vehicle first stands still for longer than the
 *         sliding window),
 *         get_geo_reference() and the YAML file written on request.
 *         A fix stamped 40 ms away from its keyframe at 10 m/s is 0.4 m away
 *         from it, so a wrong time offset sign shows up as a large ENU error.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/topography/conversions.h>

#include <cmath>
#include <iostream>
#include <optional>

#if defined(MOLA_KERNEL_NAVSTATE_FILTER_HAS_GEO_REFERENCE)
namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr double IMU_PERIOD  = 0.01;  // [s]
constexpr double POSE_PERIOD = 0.1;  // [s]
constexpr double GNSS_PERIOD = 0.2;  // [s]
constexpr double GNSS_OFFSET = 0.04;  // [s] fix stamp minus the nearest pose stamp
constexpr double STILL_TIME  = 6.0;  // [s] longer than the sliding window
constexpr double DRIVE_TIME  = 30.0;  // [s]
constexpr double SPEED       = 10.0;  // [m/s]
constexpr double POSE_NOISE  = 0.002;  // [m], [rad]
constexpr double GNSS_SIGMA  = 0.03;  // [m]
constexpr double HEADING_DEG = 147.0;  // initial vehicle azimuth (ENU yaw)
constexpr double MAX_ENU_ERR = 0.25;  // [m]

const auto ANTENNA = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0.2, 0.9, 0, 0, 0);

const char* navStateParams = R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 4.0
    relative_pose_increment_sigma_lin: 0.005
    relative_pose_increment_sigma_ang: 0.0005
    max_time_to_use_velocity_model: 2.0
    sigma_random_walk_acceleration_linear: 1.0
    sigma_random_walk_acceleration_angular: 1.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.10
    estimate_geo_reference: true
    convergence_max_position_sigma: 1.0
    convergence_max_orientation_sigma_deg: 5.0
    keep_finalized_trajectory: true
    async_backend: false
)###";

// Vehicle pose in a local ENU frame: still, then SPEED with a gentle turn.
mrpt::poses::CPose3D gtEnuAt(double t)
{
    const double yaw0 = mrpt::DEG2RAD(HEADING_DEG);
    const double tm   = std::max(0.0, t - STILL_TIME);
    const double w    = 0.02;  // [rad/s]
    const double r    = SPEED / w;
    const double dx   = r * std::sin(w * tm);
    const double dy   = r * (1.0 - std::cos(w * tm));
    return mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        std::cos(yaw0) * dx - std::sin(yaw0) * dy, std::sin(yaw0) * dx + std::cos(yaw0) * dy, 0,
        yaw0 + w * tm, 0, 0);
}

mrpt::topography::TGeodeticCoords toGeodetic(
    const mrpt::topography::TGeodeticCoords& origin, const mrpt::math::TPoint3D& enu)
{
    mrpt::math::TPoint3D geocentric;
    mrpt::topography::ENUToGeocentric(
        enu, origin, geocentric, mrpt::topography::TEllipsoid::Ellipsoid_WGS84());
    mrpt::topography::TGeodeticCoords geo;
    mrpt::topography::geocentricToGeodetic(geocentric, geo);
    return geo;
}

mrpt::obs::CObservationGPS gnss(
    const mrpt::topography::TGeodeticCoords& origin, const mrpt::math::TPoint3D& enu, double t)
{
    const auto geo = toGeodetic(origin, enu);

    mrpt::obs::gnss::Message_NMEA_GGA gga;
    gga.fields.latitude_degrees  = geo.lat.decimal_value;
    gga.fields.longitude_degrees = geo.lon.decimal_value;
    gga.fields.altitude_meters   = geo.height;
    gga.fields.fix_quality       = 4;

    mrpt::obs::CObservationGPS obs;
    obs.sensorLabel = "gps";
    obs.sensorPose  = ANTENNA;
    obs.timestamp   = mrpt::Clock::fromDouble(t);
    obs.setMsg(gga);
    auto& cov = obs.covariance_enu.emplace();
    cov.setDiagonal(std::vector<double>(
        {mrpt::square(GNSS_SIGMA), mrpt::square(GNSS_SIGMA), mrpt::square(2 * GNSS_SIGMA)}));
    return obs;
}

// Ground-truth vehicle position in the ENU frame of `datum`:
mrpt::math::TPoint3D gtInDatumEnu(
    double t, const mrpt::topography::TGeodeticCoords& origin,
    const mrpt::topography::TGeodeticCoords& datum)
{
    mrpt::math::TPoint3D p;
    mrpt::topography::geodeticToENU_WGS84(toGeodetic(origin, gtEnuAt(t).translation()), p, datum);
    return p;
}

void test_georef_enu_outputs()
{
    auto& rng = mrpt::random::getRandomGenerator();
    rng.randomize(4321);

    mola::state_estimation_smoother::StateEstimationSmoother est;
    if (VERBOSE)
    {
        est.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    est.initialize(mrpt::containers::yaml::FromText(navStateParams));

    const mrpt::topography::TGeodeticCoords origin(36.8281, -2.4074, 50.0);
    const auto                              T_enu_odom = gtEnuAt(0);

    const size_t nPoseSteps = static_cast<size_t>(POSE_PERIOD / IMU_PERIOD + 0.5);
    const size_t nGnssSteps = static_cast<size_t>(GNSS_PERIOD / IMU_PERIOD + 0.5);
    const size_t nGnssShift = static_cast<size_t>(GNSS_OFFSET / IMU_PERIOD + 0.5);

    std::optional<double> firstGeorefTime;
    double                maxNavstateErr = 0;
    size_t                nNavstate      = 0;

    for (size_t i = 0; IMU_PERIOD * static_cast<double>(i) <= STILL_TIME + DRIVE_TIME; i++)
    {
        const double t     = IMU_PERIOD * static_cast<double>(i);
        const auto   stamp = mrpt::Clock::fromDouble(t);

        mrpt::obs::CObservationIMU imu;
        imu.timestamp   = stamp;
        imu.sensorLabel = "imu";
        imu.set(mrpt::obs::IMU_X_ACC, rng.drawGaussian1D(0, 0.05));
        imu.set(mrpt::obs::IMU_Y_ACC, rng.drawGaussian1D(0, 0.05));
        imu.set(mrpt::obs::IMU_Z_ACC, 9.81 + rng.drawGaussian1D(0, 0.05));
        est.fuse_imu(imu);

        // Fixes stamped GNSS_OFFSET after a pose stamp:
        if (i >= nGnssShift && (i - nGnssShift) % nGnssSteps == 0)
        {
            const auto p = (gtEnuAt(t) + ANTENNA).translation();
            est.fuse_gnss(gnss(
                origin,
                {p.x + rng.drawGaussian1D(0, GNSS_SIGMA), p.y + rng.drawGaussian1D(0, GNSS_SIGMA),
                 p.z + rng.drawGaussian1D(0, 2 * GNSS_SIGMA)},
                t));
        }

        if (i % nPoseSteps != 0)
        {
            continue;
        }

        mrpt::poses::CPose3DPDFGaussian pdf;
        pdf.mean = (gtEnuAt(t) - T_enu_odom) + mrpt::poses::CPose3D::FromXYZYawPitchRoll(
                                                   rng.drawGaussian1D(0, POSE_NOISE),
                                                   rng.drawGaussian1D(0, POSE_NOISE), 0,
                                                   rng.drawGaussian1D(0, POSE_NOISE), 0, 0);
        pdf.cov.setIdentity();
        pdf.cov *= mrpt::square(POSE_NOISE);
        est.fuse_pose(stamp, pdf, "odom");

        (void)est.estimated_navstate(stamp, "map");  // runs the (synchronous) solve

        const auto g = est.get_geo_reference();
        if (!g)
        {
            // Not available while standing still (the azimuth is unobserved):
            ASSERT_(!est.estimated_navstate(stamp, "enu").has_value());
            continue;
        }
        if (!firstGeorefTime)
        {
            firstGeorefTime = t;
            std::cout << "First geo-reference at t=" << t << " s\n";
        }
        // Give the azimuth a few seconds to settle before checking positions:
        if (t < *firstGeorefTime + 3.0)
        {
            continue;
        }
        const auto ns = est.estimated_navstate(stamp, "enu");
        ASSERT_(ns.has_value());
        const auto   gt  = gtInDatumEnu(t, origin, g->geo_coord);
        const double err = (ns->pose.mean.translation() - gt).norm();
        maxNavstateErr   = std::max(maxNavstateErr, err);
        nNavstate++;
        if (VERBOSE)
        {
            std::cout << "t=" << t << " ENU navstate error=" << err << " m\n";
        }
    }

    ASSERT_(firstGeorefTime.has_value());
    ASSERT_GT_(*firstGeorefTime, STILL_TIME);
    std::cout << "Causal ENU navstate: max error " << maxNavstateErr << " m over " << nNavstate
              << " queries\n";
    ASSERT_GT_(nNavstate, 100U);
    ASSERT_LT_(maxNavstateErr, MAX_ENU_ERR);

    const auto g = est.get_geo_reference();
    ASSERT_(g.has_value());

    // Smoothed trajectory in ENU, including the keyframes finalized around the
    // moment the geo-reference became available:
    const auto traj = est.estimated_trajectory(
        mrpt::Clock::fromDouble(0.0), mrpt::Clock::fromDouble(STILL_TIME + DRIVE_TIME), "enu");
    ASSERT_(traj.has_value());
    double maxTrajErr = 0;
    for (const auto& [stamp, pose] : *traj)
    {
        const auto   gt  = gtInDatumEnu(mrpt::Clock::toDouble(stamp), origin, g->geo_coord);
        const double err = (mrpt::math::TPoint3D(pose.x, pose.y, pose.z) - gt).norm();
        maxTrajErr       = std::max(maxTrajErr, err);
        if (VERBOSE)
        {
            std::cout << "kf t=" << mrpt::Clock::toDouble(stamp) << " ENU error=" << err << " m\n";
        }
    }
    std::cout << "Smoothed ENU trajectory: max error " << maxTrajErr << " m over " << traj->size()
              << " keyframes\n";
    ASSERT_GT_(traj->size(), 100U);
    ASSERT_LT_(maxTrajErr, MAX_ENU_ERR);

    // The YAML file, readable with the mp2p_icp georeferencing keys:
    const auto file = mrpt::system::getTempFileName() + ".georef.yaml";
    ASSERT_(est.save_geo_reference_to_file(file));
    const auto d = mrpt::containers::yaml::FromFile(file);
    ASSERT_EQUAL_(d["type"].as<std::string>(), std::string("mp2p_icp::Georeferencing"));
    ASSERT_(d["defined"].as<bool>());
    ASSERT_NEAR_(d["geo_coord"]["lat"].as<double>(), g->geo_coord.lat.decimal_value, 1e-9);
    ASSERT_NEAR_(d["geo_coord"]["lon"].as<double>(), g->geo_coord.lon.decimal_value, 1e-9);
    ASSERT_NEAR_(
        d["T_enu_to_map"]["mean"]["yaw_deg"].as<double>(),
        mrpt::RAD2DEG(g->T_enu_to_map.mean.yaw()), 1e-6);
    ASSERT_NEAR_(d["T_enu_to_map"]["mean"]["x"].as<double>(), g->T_enu_to_map.mean.x(), 1e-6);
    mrpt::system::deleteFile(file);
}
}  // namespace
#endif

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
#if defined(MOLA_KERNEL_NAVSTATE_FILTER_HAS_GEO_REFERENCE)
        test_georef_enu_outputs();
#else
        std::cout << "Skipped: this mola_kernel has no NavStateFilter::get_geo_reference().\n";
#endif
        std::cout << "All tests passed." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
