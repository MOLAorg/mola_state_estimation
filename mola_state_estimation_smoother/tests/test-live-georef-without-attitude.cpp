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
 * @file   test-live-georef-without-attitude.cpp
 * @brief  Live geo-referencing (estimate_geo_reference) with odometry-frame
 *         poses, a gravity-only IMU and accurate GNSS, heading far from East:
 *         only the GNSS track observes the azimuth of T_enu_to_map. While the
 *         vehicle stands still, that azimuth is unobserved (pinned by a gauge
 *         anchor), so no geo-reference may be reported; once it moves, the
 *         reported one must follow the true azimuth, also after standing still
 *         for longer than the sliding window.
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

#include <cmath>
#include <iostream>
#include <optional>

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr double IMU_PERIOD  = 0.01;  // [s]
constexpr double POSE_PERIOD = 0.1;  // [s]
constexpr double GNSS_PERIOD = 1.0;  // [s]
constexpr double DRIVE_TIME  = 30.0;  // [s] driving after standing still
constexpr double POSE_NOISE  = 0.002;  // [m], [rad]
constexpr double GNSS_SIGMA  = 0.03;  // [m]
constexpr double HEADING_DEG = 147.0;  // initial vehicle azimuth (ENU yaw)

// GNSS antenna on the vehicle, sideways and high (as on a ground robot):
const auto ANTENNA = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0.2, 0.9, 0, 0, 0);

// sliding_window_length: 6 s
const char* navStateParams = R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 6.0
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
    async_backend: false
)###";

// Vehicle pose in ENU: still for `stillTime`, then 1 m/s with a gentle turn.
mrpt::poses::CPose3D gtEnuAt(double t, double stillTime)
{
    const double yaw0 = mrpt::DEG2RAD(HEADING_DEG);
    const double tm   = std::max(0.0, t - stillTime);
    const double w    = 0.05;  // [rad/s]
    const double r    = 1.0 / w;
    const double dx   = r * std::sin(w * tm);
    const double dy   = r * (1.0 - std::cos(w * tm));
    return mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        std::cos(yaw0) * dx - std::sin(yaw0) * dy, std::sin(yaw0) * dx + std::cos(yaw0) * dy, 0,
        yaw0 + w * tm, 0, 0);
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

void test_live_georef_without_attitude(double stillTime)
{
    std::cout << "Standing still for " << stillTime << " s:\n";

    auto& rng = mrpt::random::getRandomGenerator();
    rng.randomize(1234);

    mola::state_estimation_smoother::StateEstimationSmoother est;
    if (VERBOSE)
    {
        est.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    est.initialize(mrpt::containers::yaml::FromText(navStateParams));

    const mrpt::topography::TGeodeticCoords origin(36.8281, -2.4074, 50.0);

    // The odometry frame starts at the vehicle: {map} is that frame, so the
    // azimuth of T_enu_to_map is the initial vehicle heading.
    const auto   T_enu_odom  = gtEnuAt(0, stillTime);
    const double trueAzimuth = T_enu_odom.yaw();

    const size_t nPoseSteps = static_cast<size_t>(POSE_PERIOD / IMU_PERIOD + 0.5);
    const size_t nGnssSteps = static_cast<size_t>(GNSS_PERIOD / IMU_PERIOD + 0.5);

    std::optional<double> firstReportTime;
    double                maxReportedErr  = 0;
    double                lastReportedErr = 180.0;

    for (size_t i = 0; IMU_PERIOD * static_cast<double>(i) <= stillTime + DRIVE_TIME; i++)
    {
        const double t     = IMU_PERIOD * static_cast<double>(i);
        const auto   stamp = mrpt::Clock::fromDouble(t);

        // Gravity-only IMU (level ground), no orientation:
        mrpt::obs::CObservationIMU imu;
        imu.timestamp   = stamp;
        imu.sensorLabel = "imu";
        imu.set(mrpt::obs::IMU_X_ACC, rng.drawGaussian1D(0, 0.05));
        imu.set(mrpt::obs::IMU_Y_ACC, rng.drawGaussian1D(0, 0.05));
        imu.set(mrpt::obs::IMU_Z_ACC, 9.81 + rng.drawGaussian1D(0, 0.05));
        est.fuse_imu(imu);

        if (i % nGnssSteps == 0)
        {
            const auto p = (gtEnuAt(t, stillTime) + ANTENNA).translation();
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
        pdf.mean = (gtEnuAt(t, stillTime) - T_enu_odom) +
                   mrpt::poses::CPose3D::FromXYZYawPitchRoll(
                       rng.drawGaussian1D(0, POSE_NOISE), rng.drawGaussian1D(0, POSE_NOISE), 0,
                       rng.drawGaussian1D(0, POSE_NOISE), 0, 0);
        pdf.cov.setIdentity();
        pdf.cov *= mrpt::square(POSE_NOISE);
        est.fuse_pose(stamp, pdf, "odom");

        (void)est.estimated_navstate(stamp, "map");  // runs the (synchronous) solve

        if (const auto g = est.current_georeferencing(); g.has_value())
        {
            const double err = angle_err_deg(g->T_enu_to_map.mean.yaw(), trueAzimuth);
            if (!firstReportTime)
            {
                firstReportTime = t;
                std::cout << "  First geo-reference at t=" << t << " s, azimuth error " << err
                          << " deg\n";
            }
            maxReportedErr  = std::max(maxReportedErr, err);
            lastReportedErr = err;
            if (VERBOSE)
            {
                std::cout << "t=" << t << " T_enu_to_map=" << g->T_enu_to_map.mean.asString()
                          << " azimuth error=" << err << " deg\n";
            }
        }
    }

    std::cout << "  Azimuth error of the reported geo-reference: max " << maxReportedErr
              << " deg, last " << lastReportedErr << " deg\n";

    ASSERT_(firstReportTime.has_value());
    // Not while standing still (the fixes do not observe the azimuth yet):
    ASSERT_GT_(*firstReportTime, stillTime);
    // convergence_max_orientation_sigma_deg (5) plus margin:
    ASSERT_LT_(maxReportedErr, 10.0);
    ASSERT_LT_(lastReportedErr, 2.0);
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        // Shorter and longer than the sliding window:
        test_live_georef_without_attitude(3.0);
        test_live_georef_without_attitude(10.0);
        std::cout << "All tests passed." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
