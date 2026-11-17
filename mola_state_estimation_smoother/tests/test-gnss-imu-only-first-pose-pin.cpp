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
 * @file   test-gnss-imu-only-first-pose-pin.cpp
 * @brief  Live geo-referencing from GNSS and an IMU with absolute attitude alone
 *         (no pose or odometry source), with link_first_pose_to_reference_origin_sigma
 *         defining {map} as where the vehicle starts, as the ROS 2 launch file sets
 *         it, and the IMU starting before the GNSS: the pin must stay on the first
 *         keyframe, every solve must succeed, the geo-reference must converge to the
 *         truth, and the vehicle must stay on its track in {map}.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/topography/conversions.h>

#include <algorithm>
#include <cmath>
#include <iostream>

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr double IMU_PERIOD  = 0.01;  // [s]
constexpr double GNSS_PERIOD = 0.2;  // [s]
constexpr double GNSS_START  = 1.0;  // [s] the IMU starts first
constexpr double STILL_TIME  = 3.0;  // [s]
constexpr double DRIVE_TIME  = 30.0;  // [s]
constexpr double RAMP_TIME   = 2.0;  // [s] from standing to SPEED
constexpr double SPEED       = 2.0;  // [m/s]
constexpr double CURVATURE   = 0.02;  // [1/m]
constexpr double GNSS_SIGMA  = 0.03;  // [m]
constexpr double HEADING_DEG = 60.0;  // initial vehicle azimuth (ENU yaw)
constexpr double G           = 9.81;  // [m/s^2]

const char* PARAMS_YAML = R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 6.0
    max_time_to_use_velocity_model: 2.0
    estimate_geo_reference: true
    convergence_max_position_sigma: 1.0
    convergence_max_orientation_sigma_deg: 5.0
    link_first_pose_to_reference_origin_sigma: 1e-6
    imu_attitude_azimuth_offset_deg: -90
    imu_attitude_sigma_deg: 2.0
    async_backend: false
)###";

// Distance along the track, and speed, after `tm` seconds of driving:
std::pair<double, double> trackAt(double tm)
{
    if (tm <= 0)
    {
        return {0, 0};
    }
    if (tm < RAMP_TIME)
    {
        const double a = SPEED / RAMP_TIME;
        return {0.5 * a * tm * tm, a * tm};
    }
    return {0.5 * SPEED * RAMP_TIME + SPEED * (tm - RAMP_TIME), SPEED};
}

// Vehicle pose in ENU: still, then along a circle of the given curvature.
mrpt::poses::CPose3D gtEnuAt(double t)
{
    const double yaw0 = mrpt::DEG2RAD(HEADING_DEG);
    const double s    = trackAt(t - STILL_TIME).first;
    const double r    = 1.0 / CURVATURE;
    const double dx   = r * std::sin(CURVATURE * s);
    const double dy   = r * (1.0 - std::cos(CURVATURE * s));
    return mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        std::cos(yaw0) * dx - std::sin(yaw0) * dy, std::sin(yaw0) * dx + std::cos(yaw0) * dy, 0,
        yaw0 + CURVATURE * s, 0, 0);
}

mrpt::obs::CObservationGPS gnss(
    const mrpt::topography::TGeodeticCoords& origin, const mrpt::math::TPoint3D& enu, double t)
{
    mrpt::math::TPoint3D geocentric;
    mrpt::topography::ENUToGeocentric(
        enu, origin, geocentric, mrpt::topography::TEllipsoid::Ellipsoid_WGS84());
    mrpt::topography::TGeodeticCoords geo;
    mrpt::topography::geocentricToGeodetic(geocentric, geo);

    mrpt::obs::gnss::Message_NMEA_GGA gga;
    gga.fields.latitude_degrees  = geo.lat.decimal_value;
    gga.fields.longitude_degrees = geo.lon.decimal_value;
    gga.fields.altitude_meters   = geo.height;
    gga.fields.fix_quality       = 4;

    mrpt::obs::CObservationGPS obs;
    obs.sensorLabel = "gps";
    obs.timestamp   = mrpt::Clock::fromDouble(t);
    obs.setMsg(gga);
    auto& cov = obs.covariance_enu.emplace();
    cov.setDiagonal(std::vector<double>(
        {mrpt::square(GNSS_SIGMA), mrpt::square(GNSS_SIGMA), mrpt::square(2 * GNSS_SIGMA)}));
    return obs;
}

double angle_err_deg(double a, double b)
{
    return std::abs(mrpt::RAD2DEG(mrpt::math::wrapToPi(a - b)));
}

void run_test()
{
    auto& rng = mrpt::random::getRandomGenerator();
    rng.randomize(1234);

    mola::state_estimation_smoother::StateEstimationSmoother est;
    if (VERBOSE)
    {
        est.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    est.initialize(mrpt::containers::yaml::FromText(PARAMS_YAML));

    const mrpt::topography::TGeodeticCoords origin(36.8281, -2.4074, 50.0);

    // {map} is where the vehicle starts (pinned first pose):
    const auto   T_enu_map   = gtEnuAt(0);
    const double trueAzimuth = T_enu_map.yaw();

    const size_t nGnssSteps = static_cast<size_t>(GNSS_PERIOD / IMU_PERIOD + 0.5);

    double lastAzimuthErr = 180.0;
    double maxPositionErr = 0;
    size_t numConverged   = 0;

    for (size_t i = 0; IMU_PERIOD * static_cast<double>(i) <= STILL_TIME + DRIVE_TIME; i++)
    {
        const double t     = IMU_PERIOD * static_cast<double>(i);
        const auto   stamp = mrpt::Clock::fromDouble(t);

        // Level ground: tangential and centripetal acceleration plus gravity
        // (read "up"), and the turn rate:
        const double tm         = t - STILL_TIME;
        const double v          = trackAt(tm).second;
        const double tangential = (tm > 0 && tm < RAMP_TIME) ? SPEED / RAMP_TIME : 0.0;

        mrpt::obs::CObservationIMU imu;
        imu.timestamp   = stamp;
        imu.sensorLabel = "imu";
        imu.set(mrpt::obs::IMU_X_ACC, tangential + rng.drawGaussian1D(0, 0.02));
        imu.set(mrpt::obs::IMU_Y_ACC, CURVATURE * v * v + rng.drawGaussian1D(0, 0.02));
        imu.set(mrpt::obs::IMU_Z_ACC, G + rng.drawGaussian1D(0, 0.02));
        imu.set(mrpt::obs::IMU_WX, rng.drawGaussian1D(0, 0.001));
        imu.set(mrpt::obs::IMU_WY, rng.drawGaussian1D(0, 0.001));
        imu.set(mrpt::obs::IMU_WZ, CURVATURE * v + rng.drawGaussian1D(0, 0.001));
        // Attitude in ENU (see imu_attitude_azimuth_offset_deg above):
        mrpt::math::CQuaternionDouble q;
        gtEnuAt(t).getAsQuaternion(q);
        imu.set(mrpt::obs::IMU_ORI_QUAT_W, q.r());
        imu.set(mrpt::obs::IMU_ORI_QUAT_X, q.x());
        imu.set(mrpt::obs::IMU_ORI_QUAT_Y, q.y());
        imu.set(mrpt::obs::IMU_ORI_QUAT_Z, q.z());
        est.fuse_imu(imu);

        if (t < GNSS_START || i % nGnssSteps != 0)
        {
            continue;
        }
        const auto p = gtEnuAt(t).translation();
        est.fuse_gnss(gnss(
            origin,
            {p.x + rng.drawGaussian1D(0, GNSS_SIGMA), p.y + rng.drawGaussian1D(0, GNSS_SIGMA),
             p.z + rng.drawGaussian1D(0, 2 * GNSS_SIGMA)},
            t));

        const auto ns = est.estimated_navstate(stamp, "map");
        if (!ns.has_value())
        {
            continue;
        }
        if (const auto g = est.current_georeferencing(); g.has_value())
        {
            numConverged++;
            lastAzimuthErr   = angle_err_deg(g->T_enu_to_map.mean.yaw(), trueAzimuth);
            const auto gtMap = gtEnuAt(t) - T_enu_map;
            maxPositionErr   = std::max(
                  maxPositionErr, (ns->pose.mean.translation() - gtMap.translation()).norm());
            if (VERBOSE)
            {
                std::cout << "t=" << t << " T_enu_to_map=" << g->T_enu_to_map.mean.asString()
                          << " vehicle in map=" << ns->pose.mean.asString()
                          << " truth=" << gtMap.asString() << "\n";
            }
        }
    }

    std::cout << "Solve failures: " << est.solve_failures()
              << ", converged estimates: " << numConverged
              << ", last azimuth error: " << lastAzimuthErr
              << " deg, max vehicle position error in {map} once converged: " << maxPositionErr
              << " m\n";

    ASSERT_EQUAL_(est.solve_failures(), 0U);
    ASSERT_GT_(numConverged, 10U);
    ASSERT_LT_(lastAzimuthErr, 2.0);
    ASSERT_LT_(maxPositionErr, 1.0);
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        run_test();
        std::cout << "All tests passed." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
