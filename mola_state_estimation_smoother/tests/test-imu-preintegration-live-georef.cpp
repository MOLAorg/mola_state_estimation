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
 * @file   test-imu-preintegration-live-georef.cpp
 * @brief  Live geo-referencing (estimate_geo_reference) with IMU preintegration:
 *         odometry, a full IMU (accelerometer and gyroscope, consistent with the
 *         motion) and GNSS, heading far from East. Until the GNSS track observes
 *         the azimuth of T_enu_to_map, it is pinned by a gauge anchor far from
 *         the true one, for longer than the sliding window (the vehicle stands
 *         still first); the reported geo-reference must still follow the true
 *         azimuth once the vehicle moves, and the vehicle must stay on its track.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/topography/conversions.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr double IMU_PERIOD  = 0.01;  // [s]
constexpr double POSE_PERIOD = 0.1;  // [s]
constexpr double GNSS_PERIOD = 1.0;  // [s]
constexpr double STILL_TIME  = 12.0;  // [s] twice the sliding window
constexpr double DRIVE_TIME  = 40.0;  // [s]
constexpr double RAMP_TIME   = 2.0;  // [s] from standing to SPEED
constexpr double SPEED       = 1.0;  // [m/s]
constexpr double CURVATURE   = 0.05;  // [1/m]
constexpr double POSE_NOISE  = 0.002;  // [m], [rad]
constexpr double GNSS_SIGMA  = 0.03;  // [m]
constexpr double HEADING_DEG = 147.0;  // initial vehicle azimuth (ENU yaw)
constexpr double G           = 9.81;  // [m/s^2]

const auto ANTENNA = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0.2, 0.9, 0, 0, 0);

const char* navStateParams = R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 6.0
    relative_pose_increment_sigma_lin: 0.02
    relative_pose_increment_sigma_ang: 0.0005
    max_time_to_use_velocity_model: 2.0
    estimate_geo_reference: true
    convergence_max_position_sigma: 1.0
    convergence_max_orientation_sigma_deg: 5.0
    async_backend: false
    imu_preintegration:
        enabled: true
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
    obs.sensorPose  = ANTENNA;
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

void test_imu_preintegration_live_georef()
{
    auto& rng = mrpt::random::getRandomGenerator();
    rng.randomize(1234);

    mola::state_estimation_smoother::StateEstimationSmoother est;
    if (VERBOSE)
    {
        est.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    est.initialize(mrpt::containers::yaml::FromText(navStateParams));

    const mrpt::topography::TGeodeticCoords origin(36.8281, -2.4074, 50.0);

    // {map} is the odometry frame, which starts at the vehicle: the azimuth of
    // T_enu_to_map is the initial vehicle heading.
    const auto   T_enu_odom  = gtEnuAt(0);
    const double trueAzimuth = T_enu_odom.yaw();

    const size_t nPoseSteps = static_cast<size_t>(POSE_PERIOD / IMU_PERIOD + 0.5);
    const size_t nGnssSteps = static_cast<size_t>(GNSS_PERIOD / IMU_PERIOD + 0.5);

    double maxReportedErr  = 0;
    double lastReportedErr = 180.0;
    double maxPositionErr  = 0;

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
        est.fuse_imu(imu);

        if (i % nGnssSteps == 0)
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

        const auto gtMap = gtEnuAt(t) - T_enu_odom;

        mrpt::poses::CPose3DPDFGaussian pdf;
        pdf.mean = gtMap + mrpt::poses::CPose3D::FromXYZYawPitchRoll(
                               rng.drawGaussian1D(0, POSE_NOISE), rng.drawGaussian1D(0, POSE_NOISE),
                               0, rng.drawGaussian1D(0, POSE_NOISE), 0, 0);
        pdf.cov.setIdentity();
        pdf.cov *= mrpt::square(POSE_NOISE);
        est.fuse_pose(stamp, pdf, "odom");

        const auto ns = est.estimated_navstate(stamp, "map");
        if (ns.has_value())
        {
            maxPositionErr = std::max(
                maxPositionErr, (ns->pose.mean.translation() - gtMap.translation()).norm());
        }

        if (const auto g = est.current_georeferencing(); g.has_value())
        {
            const double err = angle_err_deg(g->T_enu_to_map.mean.yaw(), trueAzimuth);
            maxReportedErr   = std::max(maxReportedErr, err);
            lastReportedErr  = err;
            if (VERBOSE)
            {
                std::cout << "t=" << t << " T_enu_to_map=" << g->T_enu_to_map.mean.asString()
                          << " azimuth error=" << err << " deg\n";
            }
        }
    }

    std::cout << "Azimuth error of the reported geo-reference: max " << maxReportedErr
              << " deg, last " << lastReportedErr << " deg; vehicle position error in {map}: max "
              << maxPositionErr << " m\n";

    // convergence_max_orientation_sigma_deg (5) plus margin:
    ASSERT_LT_(maxReportedErr, 10.0);
    ASSERT_LT_(lastReportedErr, 2.0);
    ASSERT_LT_(maxPositionErr, 0.5);
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_imu_preintegration_live_georef();
        std::cout << "All tests passed." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
