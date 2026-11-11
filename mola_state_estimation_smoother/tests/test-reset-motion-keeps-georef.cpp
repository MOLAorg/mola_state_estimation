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
 * @file   test-reset-motion-keeps-georef.cpp
 * @brief  reset_motion() keeps a converged estimated geo-reference (same datum,
 *         published all along) while {map} is the same frame after it, and drops
 *         it when {map} is defined again elsewhere. A geo-reference that had not
 *         converged, or a fixed one, is not affected.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 11, 2026
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
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
#include <string>

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr double IMU_PERIOD  = 0.01;  // [s]
constexpr double POSE_PERIOD = 0.1;  // [s]
constexpr double GNSS_PERIOD = 1.0;  // [s]
constexpr double T_RESET     = 15.0;  // [s]
constexpr double T_END       = 25.0;  // [s]
constexpr double POSE_NOISE  = 0.002;  // [m], [rad]
constexpr double GNSS_SIGMA  = 0.03;  // [m]
constexpr double HEADING_DEG = 147.0;  // initial vehicle azimuth (ENU yaw)

const auto ANTENNA = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0.2, 0.9, 0, 0, 0);

const mrpt::topography::TGeodeticCoords ORIGIN(36.8281, -2.4074, 50.0);

std::string params(bool estimate)
{
    return std::string(R"###(
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
    convergence_max_position_sigma: 1.0
    convergence_max_orientation_sigma_deg: 5.0
    async_backend: false
    estimate_geo_reference: )###") +
           (estimate ? "true" : "false") + "\n";
}

// Vehicle pose in ENU: 1 m/s with a gentle turn.
mrpt::poses::CPose3D gt_enu(double t)
{
    const double yaw0 = mrpt::DEG2RAD(HEADING_DEG);
    const double w    = 0.05;  // [rad/s]
    const double r    = 1.0 / w;
    const double dx   = r * std::sin(w * t);
    const double dy   = r * (1.0 - std::cos(w * t));
    return mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        std::cos(yaw0) * dx - std::sin(yaw0) * dy, std::sin(yaw0) * dx + std::cos(yaw0) * dy, 0,
        yaw0 + w * t, 0, 0);
}

mrpt::obs::CObservationGPS gnss(const mrpt::math::TPoint3D& enu, double t)
{
    mrpt::math::TPoint3D geocentric;
    mrpt::topography::ENUToGeocentric(
        enu, ORIGIN, geocentric, mrpt::topography::TEllipsoid::Ellipsoid_WGS84());
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

bool same_datum(const mola::Georeferencing& a, const mola::Georeferencing& b)
{
    return std::abs(a.geo_coord.lat.decimal_value - b.geo_coord.lat.decimal_value) < 1e-9 &&
           std::abs(a.geo_coord.lon.decimal_value - b.geo_coord.lon.decimal_value) < 1e-9 &&
           std::abs(a.geo_coord.height - b.geo_coord.height) < 1e-6;
}

enum class AfterReset
{
    /// The same odometry source goes on: {map} is kept
    SameSource,
    /// Only a new source goes on, its frame starting at the vehicle: {map} is
    /// defined again there once no kept frame came back for a window length
    NewSourceAtVehicle,
    /// Same, with that frame far away from the vehicle
    NewSourceFarAway
};

struct Result
{
    std::optional<mola::Georeferencing> beforeReset;
    /// Solves after the reset reporting a geo-reference with the same datum
    size_t keptReports = 0;
    /// ...with no geo-reference
    size_t noReports = 0;
    /// A report with the same datum after one with none
    bool keptAfterDrop = false;
    /// Largest error of the vehicle position in ENU after the reset [m]
    double maxEnuErr = 0;
    /// Error of the reported azimuth of T_enu_to_map at the end [deg]
    double finalAzimuthErr = 180;
};

Result run(AfterReset after, double tReset, bool estimate = true)
{
    auto& rng = mrpt::random::getRandomGenerator();
    rng.randomize(1234);

    mola::state_estimation_smoother::StateEstimationSmoother est;
    if (VERBOSE)
    {
        est.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    est.initialize(mrpt::containers::yaml::FromText(params(estimate)));

    // {map} is the frame of the first odometry source, which starts at the vehicle:
    const auto T_enu_map = gt_enu(0);
    if (!estimate)
    {
        mola::Georeferencing g;
        g.geo_coord         = ORIGIN;
        g.T_enu_to_map.mean = T_enu_map;
        g.T_enu_to_map.cov.setDiagonal(1e-6);
        est.set_geo_reference(g);
    }
    // The frame of the new source, in ENU:
    const auto T_enu_new = after == AfterReset::NewSourceFarAway
                               ? gt_enu(tReset) + mrpt::poses::CPose3D::FromXYZYawPitchRoll(
                                                      -80.0, 40.0, 0, mrpt::DEG2RAD(60.0), 0, 0)
                               : gt_enu(tReset);

    const size_t nPoseSteps = static_cast<size_t>(POSE_PERIOD / IMU_PERIOD + 0.5);
    const size_t nGnssSteps = static_cast<size_t>(GNSS_PERIOD / IMU_PERIOD + 0.5);

    Result res;
    bool   resetDone = false;
    for (size_t i = 0; IMU_PERIOD * static_cast<double>(i) <= T_END; i++)
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
            const auto p = (gt_enu(t) + ANTENNA).translation();
            est.fuse_gnss(gnss(
                {p.x + rng.drawGaussian1D(0, GNSS_SIGMA), p.y + rng.drawGaussian1D(0, GNSS_SIGMA),
                 p.z + rng.drawGaussian1D(0, 2 * GNSS_SIGMA)},
                t));
        }
        if (i % nPoseSteps != 0)
        {
            continue;
        }

        if (!resetDone && t >= tReset - 1e-9)
        {
            res.beforeReset = est.current_georeferencing();
            est.reset_motion();
            resetDone = true;
        }

        const bool                      newSource  = resetDone && after != AfterReset::SameSource;
        const auto                      T_enu_odom = newSource ? T_enu_new : T_enu_map;
        mrpt::poses::CPose3DPDFGaussian pdf;
        pdf.mean = (gt_enu(t) - T_enu_odom) + mrpt::poses::CPose3D::FromXYZYawPitchRoll(
                                                  rng.drawGaussian1D(0, POSE_NOISE),
                                                  rng.drawGaussian1D(0, POSE_NOISE), 0,
                                                  rng.drawGaussian1D(0, POSE_NOISE), 0, 0);
        pdf.cov.setIdentity();
        pdf.cov *= mrpt::square(POSE_NOISE);
        est.fuse_pose(stamp, pdf, newSource ? "odom2" : "odom");

        (void)est.estimated_navstate(stamp, "map");  // runs the (synchronous) solve
        if (!resetDone)
        {
            continue;
        }

        const auto g = est.current_georeferencing();
        if (!g)
        {
            res.noReports++;
        }
        else if (res.beforeReset && same_datum(*g, *res.beforeReset))
        {
            res.keptReports++;
            res.keptAfterDrop = res.keptAfterDrop || res.noReports > 0;
        }
        if (g)
        {
            res.finalAzimuthErr = std::abs(
                mrpt::RAD2DEG(mrpt::math::wrapToPi(g->T_enu_to_map.mean.yaw() - T_enu_map.yaw())));
        }
        if (const auto enu = est.estimated_navstate(stamp, "enu"); enu && g)
        {
            // The vehicle in the ENU frame of the reported datum:
            mrpt::math::TPoint3D gtInDatum;
            {
                mrpt::math::TPoint3D geocentric;
                mrpt::topography::ENUToGeocentric(
                    gt_enu(t).translation(), ORIGIN, geocentric,
                    mrpt::topography::TEllipsoid::Ellipsoid_WGS84());
                mrpt::topography::geocentricToENU_WGS84(geocentric, gtInDatum, g->geo_coord);
            }
            res.maxEnuErr = std::max(
                res.maxEnuErr,
                std::hypot(enu->pose.mean.x() - gtInDatum.x, enu->pose.mean.y() - gtInDatum.y));
        }
    }
    return res;
}

void print(const char* name, const Result& r)
{
    std::cout << "[" << name << "] converged before the reset: " << (r.beforeReset ? "yes" : "no")
              << ", solves after it with the same datum: " << r.keptReports
              << ", with no geo-reference: " << r.noReports << ", max ENU error: " << r.maxEnuErr
              << " m, final azimuth error " << r.finalAzimuthErr << " deg\n";
}

void test_kept_while_map_is_the_same()
{
    const auto r = run(AfterReset::SameSource, T_RESET);
    print("same source", r);
    ASSERT_(r.beforeReset.has_value());
    // Published all along, in the same ENU frame:
    ASSERT_EQUAL_(r.noReports, 0U);
    ASSERT_GT_(r.keptReports, 50U);
    ASSERT_LT_(r.maxEnuErr, 0.3);
    ASSERT_LT_(r.finalAzimuthErr, 2.0);
}

void test_dropped_when_map_is_defined_again(AfterReset after, const char* name)
{
    const auto r = run(after, T_RESET);
    print(name, r);
    ASSERT_(r.beforeReset.has_value());
    // Kept until {map} is defined again, then dropped for good:
    ASSERT_GT_(r.noReports, 0U);
    ASSERT_(!r.keptAfterDrop);
}

void test_not_kept_if_not_converged()
{
    const auto r = run(AfterReset::SameSource, 0.5);
    print("not converged", r);
    ASSERT_(!r.beforeReset.has_value());
    ASSERT_GT_(r.noReports, 0U);
}

void test_fixed_georef_unaffected()
{
    const auto r = run(AfterReset::SameSource, T_RESET, false /*fixed*/);
    print("fixed", r);
    ASSERT_(r.beforeReset.has_value());
    ASSERT_EQUAL_(r.noReports, 0U);
    ASSERT_LT_(r.finalAzimuthErr, 1e-3);
}

}  // namespace

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;
    try
    {
        test_kept_while_map_is_the_same();
        test_dropped_when_map_is_defined_again(AfterReset::NewSourceAtVehicle, "new frame at ~0");
        test_dropped_when_map_is_defined_again(AfterReset::NewSourceFarAway, "new frame far away");
        test_not_kept_if_not_converged();
        test_fixed_georef_unaffected();
        std::cout << "Test passed." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed:\n" << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
