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
 * @file   test-gnss-innovation-gate.cpp
 * @brief  GNSS admission tests and innovation gate of the smoother, on a
 *         simulated vehicle with odometry, a gravity-only IMU and GNSS, in
 *         realistic failure situations:
 *         - nominal RTK (also at 20 m/s) and nominal single-point fixes: (almost)
 *           nothing rejected;
 *         - a second, bogus source on the same input publishing the datum
 *           lat/lon at zero height, interleaved with the real fixes;
 *         - the same bogus source winning the start-up race (first fixes);
 *         - isolated multipath jumps;
 *         - a GNSS outage during which the odometry heading drifts, so the
 *           returning fixes are far from the estimate (must be reacquired);
 *         - the odometry heading drifting away from fixes that keep coming
 *           (must be reacquired at once), and a wrong RTK fix moving with the
 *           vehicle (must not be taken as such a drift: not with a degraded
 *           fix in between, nor with a second, degraded receiver);
 *         - an odometry jump with correct fixes (reacquired), height-only
 *           spikes and a slowly drifting fix while standing still (rejected),
 *           and correct fixes back after the estimate followed a wrong one;
 *         - the bogus source alone after the real receiver goes silent (must
 *           never be reacquired while the vehicle moves), also before the
 *           geo-reference converges;
 *         - malformed readings and quality filters;
 *         - after RTK, a stretch of degraded (float) fixes, biased by meters
 *           while reporting 0.5 m: rejected by their sigma relative to the
 *           best one seen;
 *         - the optional height band and redescending kernels;
 *         - a fixed geo-reference (relocalization).
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/format.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/topography/conversions.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <thread>

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

using mola::state_estimation_smoother::StateEstimationSmoother;

constexpr double IMU_PERIOD  = 0.01;  // [s]
constexpr double POSE_PERIOD = 0.1;  // [s]
constexpr double GNSS_PERIOD = 0.2;  // [s], by default
constexpr double HEADING_DEG = 60.0;  // initial vehicle azimuth (ENU yaw)

const mrpt::topography::TGeodeticCoords ORIGIN(36.8281, -2.4074, 130.0);

const auto ANTENNA = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0.2, 0.9, 0, 0, 0);

const std::string BASE_PARAMS = R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 4.0
    relative_pose_increment_sigma_lin: 0.01
    relative_pose_increment_sigma_ang: 0.001
    max_time_to_use_velocity_model: 2.0
    sigma_random_walk_acceleration_linear: 1.0
    sigma_random_walk_acceleration_angular: 1.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.10
)###";

/// What a fix generator may do at a given time, besides publishing the truth.
enum class FixKind
{
    Real,
    None,  //!< outage
    Impostor,  //!< datum lat/lon at zero height
    Spike,  //!< multipath jump
    Degraded,  //!< biased by meters, reporting a moderate sigma (RTK float)
    WrongFix,  //!< a constant offset of meters, with a precise sigma (wrong RTK fix)
    WrongFloat,  //!< the same offset, reporting a moderate sigma (RTK float after it)
    WrongNoRtk,  //!< the same offset, reporting a large sigma (no RTK solution)
    Float,  //!< unbiased, reporting a moderate sigma (RTK float)
    NoRtk,  //!< unbiased, sigma 3 m
    VertSpike,  //!< precise, +20 m in height only
    NaNCov,
    ZeroCov,
};

struct Scenario
{
    std::string name;
    std::string extraParams;  //!< appended to BASE_PARAMS (indented 4)
    double      convergencePosSigma = 1.0;  // [m]
    double      convergenceOriSigma = 5.0;  // [deg]
    bool        asyncBackend        = false;
    bool        estimateGeoref      = true;
    double      duration            = 40.0;  // [s]
    double      stillTime           = 2.0;  // [s]
    double      speed               = 3.0;  // [m/s]
    double      gnssSigma           = 0.03;  // [m] (horizontal; vertical is 2x)
    int         fixQuality          = 4;
    double      gnssPeriod          = GNSS_PERIOD;  // [s]
    /// East offset of the wrong fixes [m] as a function of time.
    std::function<double(double)> wrongOffsetX = [](double) { return 7.0; };
    double odomJumpAt = -1;  //!< time of a sideways odometry jump [s] (<0: none)
    double odomJumpY  = 0;  //!< its size [m]
    double stopAt     = 1e9;  //!< the vehicle stops after moving for this long [s]

    /// Odometry heading bias [rad/s] as a function of time (default none).
    std::function<double(double)>       odomYawBias = [](double) { return 0.0; };
    std::function<FixKind(double, int)> fixKind     = [](double, int) { return FixKind::Real; };
    /// Odometry and IMU are published at time t (default: always).
    std::function<bool(double)> sensorsOn = [](double) { return true; };
};

struct Result
{
    StateEstimationSmoother::GnssGateStats stats;
    double                                 maxHorizErr = 0;  //!< after the check start time [m]
    double                                 maxVertErr  = 0;  //!< after the check start time [m]
    double                                 lastErr     = std::numeric_limits<double>::infinity();
    size_t                                 nChecked    = 0;  //!< ENU poses checked
    std::optional<mrpt::topography::TGeodeticCoords> datum;
};

mrpt::poses::CPose3D gtEnuAt(const Scenario& sc, double t)
{
    const double yaw0 = mrpt::DEG2RAD(HEADING_DEG);
    const double tm   = std::min(sc.stopAt, std::max(0.0, t - sc.stillTime));
    const double w    = 0.03;  // [rad/s]
    const double r    = sc.speed / w;
    const double dx   = r * std::sin(w * tm);
    const double dy   = r * (1.0 - std::cos(w * tm));
    return mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        std::cos(yaw0) * dx - std::sin(yaw0) * dy, std::sin(yaw0) * dx + std::cos(yaw0) * dy, 0,
        yaw0 + w * tm, 0, 0);
}

mrpt::topography::TGeodeticCoords toGeodetic(const mrpt::math::TPoint3D& enu)
{
    mrpt::math::TPoint3D geocentric;
    mrpt::topography::ENUToGeocentric(
        enu, ORIGIN, geocentric, mrpt::topography::TEllipsoid::Ellipsoid_WGS84());
    mrpt::topography::TGeodeticCoords geo;
    mrpt::topography::geocentricToGeodetic(geocentric, geo);
    return geo;
}

mrpt::obs::CObservationGPS makeFix(
    const mrpt::topography::TGeodeticCoords& geo, double t, double sigma, int quality,
    double varOverride = -1)
{
    mrpt::obs::gnss::Message_NMEA_GGA gga;
    gga.fields.latitude_degrees  = geo.lat.decimal_value;
    gga.fields.longitude_degrees = geo.lon.decimal_value;
    gga.fields.altitude_meters   = geo.height;
    gga.fields.fix_quality       = static_cast<uint8_t>(quality);

    mrpt::obs::CObservationGPS obs;
    obs.sensorLabel = "gps";
    obs.sensorPose  = ANTENNA;
    obs.timestamp   = mrpt::Clock::fromDouble(t);
    obs.setMsg(gga);
    auto& cov = obs.covariance_enu.emplace();
    if (varOverride >= 0 || std::isnan(varOverride))
    {
        cov.setDiagonal(std::vector<double>({varOverride, varOverride, varOverride}));
    }
    else
    {
        cov.setDiagonal(std::vector<double>(
            {mrpt::square(sigma), mrpt::square(sigma), mrpt::square(2 * sigma)}));
    }
    return obs;
}

Result run(const Scenario& sc, double checkFrom)
{
    std::cout << "--- " << sc.name << "\n";

    auto& rng = mrpt::random::getRandomGenerator();
    rng.randomize(777);

    StateEstimationSmoother est;
    if (VERBOSE)
    {
        est.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    else
    {
        est.setMinLoggingLevel(mrpt::system::LVL_ERROR);
    }
    std::string params = BASE_PARAMS;
    params += sc.estimateGeoref ? "    estimate_geo_reference: true\n"
                                : "    estimate_geo_reference: false\n"
                                  "    fixed_geo_reference:\n"
                                  "        latitude_deg: 36.8281\n"
                                  "        longitude_deg: -2.4074\n"
                                  "        altitude: 130.0\n";
    params += mrpt::format(
        "    convergence_max_position_sigma: %f\n"
        "    convergence_max_orientation_sigma_deg: %f\n"
        "    async_backend: %s\n",
        sc.convergencePosSigma, sc.convergenceOriSigma, sc.asyncBackend ? "true" : "false");
    params += sc.extraParams;
    est.initialize(mrpt::containers::yaml::FromText(params));

    const size_t nPoseSteps = static_cast<size_t>(POSE_PERIOD / IMU_PERIOD + 0.5);
    const size_t nGnssSteps = static_cast<size_t>(sc.gnssPeriod / IMU_PERIOD + 0.5);

    // Odometry integrated from the true increments, plus an optional heading
    // bias rate (so the heading error accumulates, as with a biased gyro):
    mrpt::poses::CPose3D odom;
    mrpt::poses::CPose3D prevGt = gtEnuAt(sc, 0);

    Result res;
    int    fixIdx = 0;

    for (size_t i = 0; IMU_PERIOD * static_cast<double>(i) <= sc.duration; i++)
    {
        const double t     = IMU_PERIOD * static_cast<double>(i);
        const auto   stamp = mrpt::Clock::fromDouble(t);

        mrpt::obs::CObservationIMU imu;
        imu.timestamp   = stamp;
        imu.sensorLabel = "imu";
        imu.set(mrpt::obs::IMU_X_ACC, rng.drawGaussian1D(0, 0.05));
        imu.set(mrpt::obs::IMU_Y_ACC, rng.drawGaussian1D(0, 0.05));
        imu.set(mrpt::obs::IMU_Z_ACC, 9.81 + rng.drawGaussian1D(0, 0.05));
        if (sc.sensorsOn(t))
        {
            est.fuse_imu(imu);
        }

        if (i % nGnssSteps == 0)
        {
            const auto kind = sc.fixKind(t, fixIdx++);
            const auto p    = (gtEnuAt(sc, t) + ANTENNA).translation();
            const auto real = toGeodetic(
                {p.x + rng.drawGaussian1D(0, sc.gnssSigma),
                 p.y + rng.drawGaussian1D(0, sc.gnssSigma),
                 p.z + rng.drawGaussian1D(0, 2 * sc.gnssSigma)});
            switch (kind)
            {
                case FixKind::None:
                    break;
                case FixKind::Real:
                    est.fuse_gnss(makeFix(real, t, sc.gnssSigma, sc.fixQuality));
                    break;
                case FixKind::Impostor:
                {
                    auto bogus   = ORIGIN;
                    bogus.height = 0;
                    est.fuse_gnss(makeFix(bogus, t, sc.gnssSigma, sc.fixQuality));
                    break;
                }
                case FixKind::Spike:
                    est.fuse_gnss(makeFix(
                        toGeodetic({p.x + 15.0, p.y - 12.0, p.z + 4.0}), t, sc.gnssSigma,
                        sc.fixQuality));
                    break;
                case FixKind::Degraded:
                    est.fuse_gnss(makeFix(
                        toGeodetic({p.x + 2.0, p.y - 1.5, p.z + 2.5}), t, 0.5, sc.fixQuality));
                    break;
                case FixKind::WrongFix:
                case FixKind::WrongFloat:
                case FixKind::WrongNoRtk:
                {
                    const double sigma = kind == FixKind::WrongFix     ? sc.gnssSigma
                                         : kind == FixKind::WrongFloat ? 0.5
                                                                       : 3.0;
                    est.fuse_gnss(makeFix(
                        toGeodetic(
                            {p.x + sc.wrongOffsetX(t), p.y, p.z + 3.8 * sc.wrongOffsetX(t) / 7.0}),
                        t, sigma, sc.fixQuality));
                    break;
                }
                case FixKind::Float:
                    est.fuse_gnss(makeFix(real, t, 0.5, sc.fixQuality));
                    break;
                case FixKind::NoRtk:
                    est.fuse_gnss(makeFix(real, t, 3.0, sc.fixQuality));
                    break;
                case FixKind::VertSpike:
                    est.fuse_gnss(makeFix(
                        toGeodetic({p.x, p.y, p.z + 20.0}), t, sc.gnssSigma, sc.fixQuality));
                    break;
                case FixKind::NaNCov:
                    est.fuse_gnss(makeFix(
                        real, t, sc.gnssSigma, sc.fixQuality,
                        std::numeric_limits<double>::quiet_NaN()));
                    break;
                case FixKind::ZeroCov:
                    est.fuse_gnss(makeFix(real, t, sc.gnssSigma, sc.fixQuality, 0.0));
                    break;
            };
        }

        if (i % nPoseSteps != 0)
        {
            continue;
        }

        const auto gt  = gtEnuAt(sc, t);
        const auto inc = gt - prevGt;
        prevGt         = gt;
        if (sc.odomJumpAt >= 0 && t >= sc.odomJumpAt && t < sc.odomJumpAt + POSE_PERIOD - 1e-6)
        {
            odom = odom + mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, sc.odomJumpY, 0, 0, 0, 0);
        }
        odom = odom +
               mrpt::poses::CPose3D::FromXYZYawPitchRoll(
                   0, 0, 0, sc.odomYawBias(t) * POSE_PERIOD, 0, 0) +
               inc;

        mrpt::poses::CPose3DPDFGaussian pdf;
        pdf.mean = odom + mrpt::poses::CPose3D::FromXYZYawPitchRoll(
                              rng.drawGaussian1D(0, 0.002), rng.drawGaussian1D(0, 0.002), 0,
                              rng.drawGaussian1D(0, 0.0005), 0, 0);
        pdf.cov.setIdentity();
        pdf.cov *= mrpt::square(0.002);
        if (sc.sensorsOn(t))
        {
            est.fuse_pose(stamp, pdf, "odom");
        }

        (void)est.estimated_navstate(stamp, "map");  // runs the (synchronous) solve

        if (sc.asyncBackend)

        {
            // Let the backend thread keep up, as a real-time source would:

            std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }

        const auto g = est.get_geo_reference();
        if (!g)
        {
            continue;
        }
        res.datum = g->geo_coord;
        if (t < checkFrom)
        {
            continue;
        }
        const auto ns = est.estimated_navstate(stamp, "enu");
        if (!ns)
        {
            continue;
        }
        // Ground truth in the ENU frame of the estimator's datum:
        mrpt::math::TPoint3D gtInDatum;
        mrpt::topography::geodeticToENU_WGS84(
            toGeodetic(gt.translation()), gtInDatum, g->geo_coord);
        const auto d    = ns->pose.mean.translation() - gtInDatum;
        res.maxHorizErr = std::max(res.maxHorizErr, std::hypot(d.x, d.y));
        res.maxVertErr  = std::max(res.maxVertErr, std::abs(d.z));
        res.lastErr     = d.norm();
        res.nChecked++;
    }

    if (sc.asyncBackend)
    {
        // Let the backend drain its queue before reading the counters:
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    res.stats = est.gnss_gate_stats();
    // Error bounds mean nothing if no ENU pose was ever produced:
    if (checkFrom < sc.duration)
    {
        ASSERT_GT_(res.nChecked, 20U);
    }
    const auto& s = res.stats;
    std::cout << "  accepted=" << s.accepted << " rejected=" << s.rejected
              << " (admission=" << s.rejected_admission << ", innovation=" << s.rejected_innovation
              << ") reacquired=" << s.reacquired << " datum_restarts=" << s.datum_restarts
              << "\n  max horiz err=" << res.maxHorizErr << " m, max vert err=" << res.maxVertErr
              << " m, last err=" << res.lastErr << " m\n";
    return res;
}

// --- Scenarios ---

void test_nominal_rtk()
{
    Scenario sc;
    sc.name      = "Nominal RTK: nothing rejected";
    const auto r = run(sc, 15.0);
    ASSERT_EQUAL_(r.stats.rejected, 0U);
    ASSERT_GT_(r.stats.accepted, 150U);
    ASSERT_LT_(r.lastErr, 0.2);
    ASSERT_LT_(r.maxHorizErr, 0.3);
    ASSERT_LT_(r.maxVertErr, 0.5);
}

void test_nominal_rtk_fast()
{
    Scenario sc;
    sc.name      = "Nominal RTK at 20 m/s: a lagging prediction would reject good fixes";
    sc.speed     = 20.0;
    const auto r = run(sc, 15.0);
    ASSERT_EQUAL_(r.stats.rejected, 0U);
    ASSERT_GT_(r.stats.accepted, 150U);
    ASSERT_LT_(r.maxHorizErr, 0.3);
}

void test_nominal_single_point()
{
    Scenario sc;
    sc.name                = "Nominal single-point GNSS (sigma 2 m): almost nothing rejected";
    sc.gnssSigma           = 2.0;
    sc.fixQuality          = 1;
    sc.duration            = 60.0;
    sc.convergencePosSigma = 3.0;
    sc.convergenceOriSigma = 10.0;
    const auto r           = run(sc, 40.0);
    // chi2 25 with 3 DOF: ~2e-5 false rejections per fix.
    ASSERT_LT_(r.stats.rejected, 3U);
}

void test_impostor_interleaved()
{
    Scenario sc;
    sc.name = "Bogus source (datum lat/lon at h=0) interleaved after convergence";
    // Every other fix from t=15 s on comes from the bogus source:
    sc.fixKind = [](double t, int k)
    { return (t > 15.0 && k % 2 == 1) ? FixKind::Impostor : FixKind::Real; };
    const auto r = run(sc, 15.0);
    ASSERT_GT_(r.stats.rejected_innovation, 50U);
    ASSERT_EQUAL_(r.stats.reacquired, 0U);
    ASSERT_LT_(r.maxVertErr, 0.5);
    ASSERT_LT_(r.maxHorizErr, 0.3);
}

void test_impostor_interleaved_async()
{
    Scenario sc;
    sc.name         = "Same bogus source, async backend (predictions from solved keyframes only)";
    sc.asyncBackend = true;
    sc.fixKind      = [](double t, int k)
    { return (t > 15.0 && k % 2 == 1) ? FixKind::Impostor : FixKind::Real; };
    const auto r = run(sc, 1e9);
    // The async backend reorders solves, so only the gate's own counts are
    // checked: every bogus fix rejected, none reacquired.
    ASSERT_GE_(r.stats.rejected_innovation, 60U);
    ASSERT_EQUAL_(r.stats.reacquired, 0U);
}

void test_impostor_at_startup()
{
    Scenario sc;
    sc.name      = "Bogus source wins the start-up race (first 2 fixes)";
    sc.fixKind   = [](double, int k) { return k < 2 ? FixKind::Impostor : FixKind::Real; };
    const auto r = run(sc, 15.0);
    ASSERT_(r.datum.has_value());
    // The datum must come from the real receiver (~130 m), not the bogus one (0 m):
    ASSERT_LT_(std::abs(r.datum->height - ORIGIN.height), 2.0);
    ASSERT_GE_(r.stats.datum_restarts, 1U);
    ASSERT_LT_(r.maxVertErr, 0.5);
}

void test_multipath_spikes()
{
    Scenario sc;
    sc.name    = "Isolated multipath jumps (~20 m), 1 in 15 fixes";
    sc.fixKind = [](double t, int k)
    { return (t > 10.0 && k % 15 == 7) ? FixKind::Spike : FixKind::Real; };
    const auto r = run(sc, 15.0);
    ASSERT_GE_(r.stats.rejected_innovation, 8U);
    ASSERT_EQUAL_(r.stats.reacquired, 0U);
    ASSERT_LT_(r.maxHorizErr, 0.3);
}

void test_outage_with_heading_drift()
{
    Scenario sc;
    sc.name        = "40 s outage while the odometry heading drifts (estimate ~15 m off)";
    sc.duration    = 90.0;
    sc.speed       = 5.0;
    sc.odomYawBias = [](double t) { return (t > 20.0 && t < 60.0) ? mrpt::DEG2RAD(0.25) : 0.0; };
    sc.fixKind     = [](double t, int)
    { return (t > 20.0 && t < 60.0) ? FixKind::None : FixKind::Real; };
    const auto r = run(sc, 80.0);
    // The returning fixes must not be locked out: either within the gate grown
    // by the drift model, or reacquired.
    ASSERT_LT_(r.lastErr, 0.5);
    ASSERT_LT_(r.maxHorizErr, 0.5);
}

void test_large_drift_is_reacquired()
{
    Scenario sc;
    sc.name        = "Outage with a strong heading drift (estimate far off): reacquired";
    sc.duration    = 90.0;
    sc.speed       = 5.0;
    sc.odomYawBias = [](double t) { return (t > 20.0 && t < 40.0) ? mrpt::DEG2RAD(1.0) : 0.0; };
    sc.fixKind     = [](double t, int)
    { return (t > 20.0 && t < 40.0) ? FixKind::None : FixKind::Real; };
    const auto r = run(sc, 80.0);
    ASSERT_EQUAL_(r.stats.reacquired, 1U);
    ASSERT_LT_(r.lastErr, 0.5);
}

void test_drift_with_fixes_reacquired()
{
    Scenario sc;
    sc.name       = "Odometry heading drifts away from fixes that keep coming: reacquired";
    sc.duration   = 90.0;
    sc.speed      = 2.0;
    sc.gnssPeriod = 1.0;
    // Stiff odometry, so the estimate follows its drift rather than the fixes:
    sc.extraParams =
        "    relative_pose_increment_sigma_lin: 0.0005\n"
        "    relative_pose_increment_sigma_ang: 0.00002\n";
    sc.odomYawBias = [](double t) { return (t > 20.0 && t < 50.0) ? mrpt::DEG2RAD(2.0) : 0.0; };
    const auto r   = run(sc, 85.0);
    // Reacquired as soon as the fixes leave the gate, by the continuity of their
    // residual, instead of after a run of rejections:
    ASSERT_GE_(r.stats.reacquired_by_continuity, 1U);
    ASSERT_LT_(r.stats.rejected_innovation, 3U);
    ASSERT_LT_(r.maxHorizErr, 0.3);
}

void test_wrong_rtk_fix_not_a_drift()
{
    for (const bool floatBefore : {false, true})
    {
        Scenario sc;
        sc.name = std::string("A wrong RTK fix (7 m off, moving with the vehicle) for 25 s") +
                  (floatBefore ? ", entered from and left as RTK float" : "") +
                  ": not taken as a drift of the estimate";
        sc.duration   = 70.0;
        sc.speed      = 1.0;
        sc.gnssPeriod = 1.0;
        // With float before it, also as RTK float (and one fix without RTK) after it:
        sc.fixKind = [floatBefore](double t, int)
        {
            if (floatBefore && t > 41.5 && t < 42.5)
            {
                return FixKind::WrongNoRtk;
            }
            if (floatBefore && t > 42.5 && t < 55.0)
            {
                return FixKind::WrongFloat;
            }
            if (t > 30.0 && t < 55.0)
            {
                return FixKind::WrongFix;
            }
            return (floatBefore && t > 20.0 && t < 30.0) ? FixKind::Float : FixKind::Real;
        };
        const auto r = run(sc, 15.0);
        // Its jump breaks the residual continuity: at most a full run of
        // rejections reacquires it, as any fix moving with the vehicle.
        ASSERT_EQUAL_(r.stats.reacquired_by_continuity, 0U);
        ASSERT_GE_(r.stats.rejected_innovation, 9U);
        ASSERT_LT_(r.stats.reacquired, 2U);
    }
}

void test_impostor_alone_never_reacquired()
{
    Scenario sc;
    sc.name      = "Real receiver goes silent, only the bogus source remains";
    sc.duration  = 50.0;
    sc.fixKind   = [](double t, int) { return t > 15.0 ? FixKind::Impostor : FixKind::Real; };
    const auto r = run(sc, 15.0);
    ASSERT_EQUAL_(r.stats.reacquired, 0U);
    ASSERT_GT_(r.stats.consecutive_rejected, 100U);
    ASSERT_LT_(r.maxVertErr, 1.0);
}

void test_impostor_alone_before_convergence()
{
    Scenario sc;
    sc.name = "Before convergence: receiver silent for 15 s, only the bogus source; then back";
    // Thresholds no estimate reaches: there is never a prediction to gate against.
    sc.convergencePosSigma = 1e-4;
    sc.convergenceOriSigma = 1e-4;
    sc.fixKind             = [](double t, int)
    { return (t > 10.0 && t < 25.0) ? FixKind::Impostor : FixKind::Real; };
    const auto r = run(sc, 1e9);
    // The bogus source never takes over, and the real fixes are accepted again:
    ASSERT_EQUAL_(r.stats.reacquired, 0U);
    ASSERT_GT_(r.stats.rejected_innovation, 70U);
    ASSERT_EQUAL_(r.stats.consecutive_rejected, 0U);
}

void test_impostor_during_odometry_gap()
{
    Scenario sc;
    sc.name = "Before convergence: odometry and IMU stop for 4 s, only the bogus source then";
    sc.convergencePosSigma = 1e-4;
    sc.convergenceOriSigma = 1e-4;
    sc.sensorsOn           = [](double t) { return t < 10.0 || t > 14.0; };
    sc.fixKind             = [](double t, int)
    { return (t > 10.0 && t < 14.0) ? FixKind::Impostor : FixKind::Real; };
    const auto r = run(sc, 1e9);
    // Unknown motion during the gap must not open the speed bound to the bogus
    // source; the real fixes after it are accepted again directly:
    ASSERT_EQUAL_(r.stats.reacquired, 0U);
    ASSERT_GE_(r.stats.rejected_innovation, 18U);
    ASSERT_EQUAL_(r.stats.consecutive_rejected, 0U);
}

void test_malformed_and_quality()
{
    Scenario sc;
    sc.name        = "Malformed covariances and a quality filter";
    sc.extraParams = "    gnss_accepted_fix_qualities: \"4 5\"\n";
    sc.fixQuality  = 4;
    sc.fixKind     = [](double t, int k)
    {
        if (t > 10.0 && k % 20 == 3)
        {
            return FixKind::NaNCov;
        }
        if (t > 10.0 && k % 20 == 13)
        {
            return FixKind::ZeroCov;
        }
        return FixKind::Real;
    };
    const auto r = run(sc, 15.0);
    ASSERT_GE_(r.stats.rejected_admission, 14U);
    ASSERT_EQUAL_(r.stats.rejected_innovation, 0U);
    ASSERT_LT_(r.maxHorizErr, 0.3);

    Scenario q    = sc;
    q.name        = "Single-point fixes with only RTK accepted: all rejected";
    q.fixKind     = [](double, int) { return FixKind::Real; };
    q.fixQuality  = 1;
    q.duration    = 10.0;
    const auto rq = run(q, 1e9);
    ASSERT_EQUAL_(rq.stats.accepted, 0U);
    ASSERT_GT_(rq.stats.rejected_admission, 40U);
}

void test_height_band_before_convergence()
{
    Scenario sc;
    sc.name        = "Height band rejects the bogus source even before convergence";
    sc.extraParams = "    gnss_max_height_deviation: 20.0\n";
    sc.fixKind     = [](double, int k)
    { return (k > 3 && k % 2 == 1) ? FixKind::Impostor : FixKind::Real; };
    const auto r = run(sc, 15.0);
    ASSERT_GT_(r.stats.rejected_admission, 50U);
    ASSERT_LT_(r.maxVertErr, 0.5);
}

void test_redescending_kernels()
{
    for (const char* k : {"cauchy", "geman_mcclure", "dcs"})
    {
        Scenario sc;
        sc.name        = std::string("Kernel ") + k + ", spikes with the gate off";
        sc.extraParams = std::string("    gnss_robust_kernel: ") + k + "\n    gnss_gate_chi2: 0\n";
        sc.fixKind     = [](double t, int i)
        { return (t > 10.0 && i % 15 == 7) ? FixKind::Spike : FixKind::Real; };
        const auto r = run(sc, 15.0);
        ASSERT_EQUAL_(r.stats.rejected, 0U);
        ASSERT_GT_(r.stats.accepted, 150U);
        ASSERT_LT_(r.lastErr, 0.2);
        ASSERT_LT_(r.maxHorizErr, 0.5);
    }
}

Scenario degraded_after_rtk()
{
    Scenario sc;
    sc.duration = 45.0;
    sc.fixKind  = [](double t, int)
    { return (t > 20.0 && t < 32.0) ? FixKind::Degraded : FixKind::Real; };
    return sc;
}

void test_degraded_after_rtk()
{
    // Control: without the ratio test the degraded fixes pass the innovation
    // gate (their 0.5 m sigma widens it) and pull the estimate.
    Scenario ctrl    = degraded_after_rtk();
    ctrl.name        = "RTK, then 12 s of fixes biased by 3.5 m reporting 0.5 m (ratio test off)";
    ctrl.extraParams = "    gnss_max_sigma_ratio: 0.0\n";
    const auto rc    = run(ctrl, 15.0);
    ASSERT_GT_(rc.maxHorizErr + rc.maxVertErr, 0.6);

    Scenario sc    = degraded_after_rtk();
    sc.name        = "Same, rejected by their sigma relative to the best one seen";
    sc.extraParams = "    gnss_max_sigma_ratio: 2.0\n    gnss_sigma_ratio_window: 300.0\n";
    const auto r   = run(sc, 15.0);
    ASSERT_GE_(r.stats.rejected_admission, 55U);
    ASSERT_LT_(r.maxHorizErr, 0.3);
    ASSERT_LT_(r.maxVertErr, 0.5);
}

void test_single_point_with_ratio_test()
{
    Scenario sc;
    sc.name                = "Single-point GNSS (sigma 2 m) with the ratio test: nothing rejected";
    sc.gnssSigma           = 2.0;
    sc.fixQuality          = 1;
    sc.duration            = 60.0;
    sc.convergencePosSigma = 3.0;
    sc.convergenceOriSigma = 10.0;
    sc.extraParams         = "    gnss_max_sigma_ratio: 2.0\n    gnss_sigma_ratio_window: 300.0\n";
    const auto r           = run(sc, 40.0);
    ASSERT_EQUAL_(r.stats.rejected_admission, 0U);
}

void test_relocalization_impostor()
{
    Scenario sc;
    sc.name           = "Fixed geo-reference (relocalization), bogus source after convergence";
    sc.estimateGeoref = false;
    sc.fixKind        = [](double t, int k)
    { return (t > 15.0 && k % 2 == 1) ? FixKind::Impostor : FixKind::Real; };
    const auto r = run(sc, 15.0);
    ASSERT_GT_(r.stats.rejected_innovation, 50U);
    ASSERT_LT_(r.maxVertErr, 0.5);
}

void test_wrong_rtk_after_degraded_fix()
{
    Scenario sc;
    sc.name = "One fix without RTK, then a wrong RTK fix (7 m off) for 25 s: not taken as a drift";
    sc.duration   = 70.0;
    sc.speed      = 1.0;
    sc.gnssPeriod = 1.0;
    sc.fixKind    = [](double t, int)
    {
        if (t > 29.5 && t < 30.5)
        {
            return FixKind::NoRtk;
        }
        if (t > 30.5 && t < 55.0)
        {
            return FixKind::WrongFix;
        }
        return FixKind::Real;
    };
    const auto r = run(sc, 15.0);
    // Not reacquired by the continuity of its residual (a degraded fix in
    // between must not hide the jump), only after a full run, if at all:
    ASSERT_EQUAL_(r.stats.reacquired_by_continuity, 0U);
    ASSERT_GE_(r.stats.rejected_innovation, 9U);
    ASSERT_LT_(r.stats.reacquired, 2U);
}

void test_two_receivers_one_wrong()
{
    Scenario sc;
    sc.name =
        "Every other fix without RTK (e.g. a second receiver on the same input), the RTK ones "
        "7 m off for 25 s: not a drift";
    sc.duration   = 70.0;
    sc.speed      = 1.0;
    sc.gnssPeriod = 0.5;
    sc.fixKind    = [](double t, int k)
    {
        if (k % 2 == 1)
        {
            return FixKind::NoRtk;
        }
        return (t > 30.0 && t < 55.0) ? FixKind::WrongFix : FixKind::Real;
    };
    const auto r = run(sc, 15.0);
    // Not reacquired by the continuity of its residual (a degraded fix in
    // between must not hide the jump), only after a full run, if at all:
    ASSERT_EQUAL_(r.stats.reacquired_by_continuity, 0U);
    ASSERT_GE_(r.stats.rejected_innovation, 9U);
    ASSERT_LT_(r.stats.reacquired, 2U);
}

void test_height_spikes_not_reacquired()
{
    Scenario sc;
    sc.name    = "Isolated precise fixes 20 m off in height only: rejected";
    sc.fixKind = [](double t, int k)
    { return (t > 10.0 && k % 15 == 7) ? FixKind::VertSpike : FixKind::Real; };
    const auto r = run(sc, 15.0);
    ASSERT_EQUAL_(r.stats.reacquired, 0U);
    ASSERT_GE_(r.stats.rejected_innovation, 9U);
    ASSERT_LT_(r.maxVertErr, 0.2);
}

void test_wrong_rtk_with_degraded_break()
{
    Scenario sc;
    sc.name =
        "A wrong RTK fix for 25 s, with an unbiased and a wrong float in between: not a drift";
    sc.duration   = 70.0;
    sc.speed      = 1.0;
    sc.gnssPeriod = 1.0;
    sc.fixKind    = [](double t, int)
    {
        if (t > 34.5 && t < 35.5)
        {
            return FixKind::Float;
        }
        if (t > 35.5 && t < 36.5)
        {
            return FixKind::WrongFloat;
        }
        return (t > 30.0 && t < 55.0) ? FixKind::WrongFix : FixKind::Real;
    };
    const auto r = run(sc, 15.0);
    // Not reacquired by the continuity of its residual (a degraded fix in
    // between must not hide the jump), only after a full run, if at all:
    ASSERT_EQUAL_(r.stats.reacquired_by_continuity, 0U);
    ASSERT_GE_(r.stats.rejected_innovation, 9U);
    ASSERT_LT_(r.stats.reacquired, 2U);
}

void test_estimate_jump_reacquired()
{
    Scenario sc;
    sc.name     = "The odometry jumps 5 m sideways while correct RTK fixes keep coming: reacquired";
    sc.duration = 90.0;
    sc.speed    = 1.0;
    sc.gnssPeriod = 1.0;
    sc.odomJumpAt = 30.0;
    sc.odomJumpY  = 5.0;
    const auto r  = run(sc, 60.0);
    ASSERT_GE_(r.stats.reacquired, 1U);
    ASSERT_LT_(r.stats.rejected_innovation, 12U);
    ASSERT_LT_(r.maxHorizErr, 1.0);
}

void test_wrong_rtk_slow_drift_standing_still()
{
    Scenario sc;
    sc.name         = "Standing still, a precise fix drifts away at 0.3 m/s: never reacquired";
    sc.duration     = 70.0;
    sc.speed        = 1.0;
    sc.stopAt       = 23.0;
    sc.gnssPeriod   = 1.0;
    sc.wrongOffsetX = [](double t) { return std::min(7.5, 0.3 * std::max(0.0, t - 30.0)); };
    sc.fixKind      = [](double t, int)
    { return (t > 30.0 && t < 60.0) ? FixKind::WrongFix : FixKind::Real; };
    const auto r = run(sc, 15.0);
    ASSERT_EQUAL_(r.stats.reacquired, 0U);
    ASSERT_LT_(r.maxHorizErr, 2.0);
}

void test_correct_fixes_back_after_followed_fault()
{
    Scenario sc;
    sc.name =
        "Floats slide onto a wrong RTK fix the estimate follows: correct fixes are back after it";
    sc.duration     = 70.0;
    sc.speed        = 1.0;
    sc.gnssPeriod   = 1.0;
    sc.wrongOffsetX = [](double t) { return (t < 30.0) ? std::max(0.0, 0.7 * (t - 20.0)) : 7.0; };
    sc.fixKind      = [](double t, int)
    {
        if (t > 20.5 && t < 30.0)
        {
            return FixKind::WrongFloat;
        }
        return (t > 29.5 && t < 55.0) ? FixKind::WrongFix : FixKind::Real;
    };
    const auto r = run(sc, 15.0);
    ASSERT_GT_(r.maxHorizErr, 3.0);  // the estimate did follow the wrong fixes
    ASSERT_LT_(r.lastErr, 3.0);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_nominal_rtk();
        test_nominal_rtk_fast();
        test_nominal_single_point();
        test_impostor_interleaved();
        test_impostor_interleaved_async();
        test_impostor_at_startup();
        test_multipath_spikes();
        test_outage_with_heading_drift();
        test_large_drift_is_reacquired();
        test_drift_with_fixes_reacquired();
        test_wrong_rtk_fix_not_a_drift();
        test_impostor_alone_never_reacquired();
        test_impostor_alone_before_convergence();
        test_impostor_during_odometry_gap();
        test_malformed_and_quality();
        test_height_band_before_convergence();
        test_redescending_kernels();
        test_relocalization_impostor();
        test_degraded_after_rtk();
        test_single_point_with_ratio_test();
        test_wrong_rtk_after_degraded_fix();
        test_two_receivers_one_wrong();
        test_height_spikes_not_reacquired();
        test_wrong_rtk_with_degraded_break();
        test_estimate_jump_reacquired();
        test_wrong_rtk_slow_drift_standing_still();
        test_correct_fixes_back_after_followed_fault();
        std::cout << "All tests passed." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
