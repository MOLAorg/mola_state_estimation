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
 * @file   test-gnss-bias-state.cpp
 * @brief  GNSS receiver bias states (Parameters::gnss_bias_sigma_xy): a low-cost
 *         receiver with a constant offset, alone and with a precise receiver that is
 *         only available for a while.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 19, 2026
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/format.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/topography/conversions.h>

#include <cmath>
#include <iostream>
#include <map>
#include <optional>
#include <string>

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

using mola::state_estimation_smoother::StateEstimationSmoother;

constexpr double IMU_PERIOD  = 0.01;  // [s]
constexpr double POSE_PERIOD = 0.1;  // [s]
constexpr double GNSS_PERIOD = 1.0;  // [s]
constexpr double DURATION    = 90.0;  // [s]

// The low-cost receiver: a constant offset plus white noise, reporting the
// sigma of the white part only.
const mrpt::math::TPoint3D LOWCOST_BIAS(1.5, -1.0, 0.0);  // [m]
constexpr double           LOWCOST_SIGMA = 0.2;  // [m]
constexpr double           RTK_SIGMA     = 0.03;  // [m]

const mrpt::topography::TGeodeticCoords ORIGIN(36.8281, -2.4074, 130.0);

const auto LOWCOST_ANTENNA = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.1, -0.2, 0.8, 0, 0, 0);
const auto RTK_ANTENNA     = mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0.2, 0.9, 0, 0, 0);

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
    estimate_geo_reference: true
    convergence_max_position_sigma: 5.0
    convergence_max_orientation_sigma_deg: 5.0
    async_backend: false
)###";

struct Scenario
{
    std::string name;
    double      biasSigmaXY   = 0;  //!< gnss_bias_sigma_xy [m]
    double      rtkUntil      = -1;  //!< the precise receiver delivers fixes until then [s]
    double      odomScale     = 1.0;  //!< odometry distance scale (e.g. lost encoder counts)
    double      restartSigmas = 3.0;  //!< gnss_bias_restart_sigmas
    bool        lowCostBiased = true;  //!< the low-cost receiver has its constant offset
};

struct Result
{
    double                                                           finalHorizErr = 0;  // [m]
    double                                                           maxHorizErr   = 0;  // [m]
    double                                                           finalSigmaXY  = 0;  // [m]
    std::map<std::string, StateEstimationSmoother::GnssBiasEstimate> biases;
    StateEstimationSmoother::GnssGateStats                           stats;
};

/// Ground truth vehicle pose in ENU (around ORIGIN): a wide circle at 3 m/s
mrpt::poses::CPose3D gtEnuAt(double t)
{
    const double yaw0 = mrpt::DEG2RAD(60.0);
    const double tm   = std::max(0.0, t - 2.0);
    const double w    = 0.03;  // [rad/s]
    const double r    = 3.0 / w;
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
    const std::string& label, const mrpt::poses::CPose3D& antenna, const mrpt::math::TPoint3D& enu,
    double t, double sigma, int quality)
{
    const auto                        geo = toGeodetic(enu);
    mrpt::obs::gnss::Message_NMEA_GGA gga;
    gga.fields.latitude_degrees  = geo.lat.decimal_value;
    gga.fields.longitude_degrees = geo.lon.decimal_value;
    gga.fields.altitude_meters   = geo.height;
    gga.fields.fix_quality       = static_cast<uint8_t>(quality);

    mrpt::obs::CObservationGPS obs;
    obs.sensorLabel = label;
    obs.sensorPose  = antenna;
    obs.timestamp   = mrpt::Clock::fromDouble(t);
    obs.setMsg(gga);
    obs.covariance_enu.emplace().setDiagonal(
        std::vector<double>({mrpt::square(sigma), mrpt::square(sigma), mrpt::square(2 * sigma)}));
    return obs;
}

Result run(const Scenario& sc)
{
    std::cout << "--- " << sc.name << "\n";

    auto& rng = mrpt::random::getRandomGenerator();
    rng.randomize(1234);

    StateEstimationSmoother est;
    est.setMinLoggingLevel(VERBOSE ? mrpt::system::LVL_DEBUG : mrpt::system::LVL_ERROR);
    std::string params = BASE_PARAMS;
    params += mrpt::format(
        "    gnss_bias_sigma_xy: %f\n"
        "    gnss_bias_time_constant: 300.0\n"
        "    gnss_bias_labels_re: \"lowcost\"\n"
        "    gnss_bias_restart_sigmas: %f\n",
        sc.biasSigmaXY, sc.restartSigmas);
    est.initialize(mrpt::containers::yaml::FromText(params));

    const size_t nPoseSteps = static_cast<size_t>(std::lround(POSE_PERIOD / IMU_PERIOD));
    const size_t nGnssSteps = static_cast<size_t>(std::lround(GNSS_PERIOD / IMU_PERIOD));

    mrpt::poses::CPose3D odom;
    mrpt::poses::CPose3D prevGt = gtEnuAt(0);
    Result               res;

    for (size_t i = 0; IMU_PERIOD * static_cast<double>(i) <= DURATION; i++)
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

        const auto noisy = [&](const mrpt::math::TPoint3D& p, double sigma)
        {
            return mrpt::math::TPoint3D(
                p.x + rng.drawGaussian1D(0, sigma), p.y + rng.drawGaussian1D(0, sigma),
                p.z + rng.drawGaussian1D(0, 2 * sigma));
        };
        if (i % nGnssSteps == 0)
        {
            const auto p = (gtEnuAt(t) + LOWCOST_ANTENNA).translation();
            est.fuse_gnss(makeFix(
                "lowcost", LOWCOST_ANTENNA,
                noisy(sc.lowCostBiased ? p + LOWCOST_BIAS : p, LOWCOST_SIGMA), t, LOWCOST_SIGMA,
                1));
        }
        if (i % nGnssSteps == nGnssSteps / 2 && t < sc.rtkUntil)
        {
            const auto p = (gtEnuAt(t) + RTK_ANTENNA).translation();
            est.fuse_gnss(makeFix("rtk", RTK_ANTENNA, noisy(p, RTK_SIGMA), t, RTK_SIGMA, 4));
        }

        if (i % nPoseSteps != 0)
        {
            continue;
        }
        const auto gt  = gtEnuAt(t);
        const auto inc = gt - prevGt;
        prevGt         = gt;
        odom           = odom + mrpt::poses::CPose3D::FromXYZYawPitchRoll(
                                    inc.x() * sc.odomScale, inc.y() * sc.odomScale, inc.z() * sc.odomScale,
                                    inc.yaw(), inc.pitch(), inc.roll());

        mrpt::poses::CPose3DPDFGaussian pdf;
        pdf.mean = odom + mrpt::poses::CPose3D::FromXYZYawPitchRoll(
                              rng.drawGaussian1D(0, 0.002), rng.drawGaussian1D(0, 0.002), 0,
                              rng.drawGaussian1D(0, 0.0005), 0, 0);
        pdf.cov.setIdentity();
        pdf.cov *= mrpt::square(0.002);
        est.fuse_pose(stamp, pdf, "odom");

        const auto ns = est.estimated_navstate(stamp, "enu");
        const auto g  = est.get_geo_reference();
        if (!ns || !g)
        {
            continue;
        }
        mrpt::math::TPoint3D gtInDatum;
        mrpt::topography::geodeticToENU_WGS84(
            toGeodetic(gt.translation()), gtInDatum, g->geo_coord);
        mrpt::poses::CPose3DPDFGaussian pose;
        pose.copyFrom(ns->pose);
        const auto d      = pose.mean.translation() - gtInDatum;
        res.finalHorizErr = std::hypot(d.x, d.y);
        if (t > 30.0)
        {
            res.maxHorizErr = std::max(res.maxHorizErr, res.finalHorizErr);
        }
        res.finalSigmaXY = std::sqrt(std::max(pose.cov(0, 0), pose.cov(1, 1)));
    }
    res.biases = est.estimated_gnss_biases();
    res.stats  = est.gnss_gate_stats();
    ASSERT_EQUAL_(est.solve_failures(), 0U);

    std::cout << mrpt::format(
        "    final horizontal error %.3f m (max %.3f m after 30 s), reported sigma %.3f m, GNSS "
        "accepted %zu rejected %zu, bias restarts %zu\n",
        res.finalHorizErr, res.maxHorizErr, res.finalSigmaXY, res.stats.accepted,
        res.stats.rejected, res.stats.bias_restarts);
    for (const auto& [label, b] : res.biases)
    {
        std::cout << mrpt::format(
            "    bias '%s': (%.3f, %.3f, %.3f) m, sigmas (%.3f, %.3f) m\n", label.c_str(), b.mean.x,
            b.mean.y, b.mean.z, std::sqrt(b.cov(0, 0)), std::sqrt(b.cov(1, 1)));
    }
    return res;
}

// Off by default: no bias variables, and the offset that all fixes share stays
// in the estimate.
void test_off_by_default()
{
    const auto r = run({"low-cost receiver alone, no bias state", 0.0, -1});
    ASSERT_(r.biases.empty());
    ASSERT_GT_(r.finalHorizErr, 1.0);
}

// A constant offset is not observable with one receiver, but the bias state keeps
// the reported uncertainty above it, and the bias estimate as uncertain as its prior.
void test_honest_sigma()
{
    const auto r = run({"low-cost receiver alone, bias state", 2.0, -1});
    ASSERT_EQUAL_(r.biases.size(), 1U);
    ASSERT_GT_(r.finalSigmaXY, 1.0);
    ASSERT_LT_(r.finalHorizErr, 3.0 * r.finalSigmaXY);
    const auto& b = r.biases.begin()->second;
    ASSERT_GT_(std::sqrt(b.cov(0, 0)), 1.0);
}

// With a precise receiver for a while, the bias becomes observable, and the
// low-cost receiver alone keeps the estimate close to it afterwards.
void test_observable_with_rtk()
{
    const double rtkUntil = 30.0;
    const auto   off      = run({"low-cost + RTK for 30 s, no bias state", 0.0, rtkUntil});
    const auto   on       = run({"low-cost + RTK for 30 s, bias state", 2.0, rtkUntil});

    ASSERT_EQUAL_(on.biases.size(), 1U);
    ASSERT_EQUAL_(on.biases.count("lowcost"), 1U);
    const auto& b = on.biases.at("lowcost");
    ASSERT_LT_(std::hypot(b.mean.x - LOWCOST_BIAS.x, b.mean.y - LOWCOST_BIAS.y), 0.3);
    ASSERT_LT_(on.finalHorizErr, 0.5);
    ASSERT_GT_(off.finalHorizErr, 1.0);
}

// Odometry that under-reports the distance (as with lost encoder counts): the bias
// chain absorbs the drift (about drift rate * tau), unless it restarts once the
// bias is implausible for the receiver.
void test_restart_on_odometry_drift()
{
    Scenario sc{"drifting odometry, bias state, no restarts", 2.0, -1};
    sc.odomScale           = 0.8;
    sc.lowCostBiased       = false;
    sc.restartSigmas       = 0;
    const auto noRestart   = run(sc);
    sc.name                = "drifting odometry, bias state, restarts";
    sc.restartSigmas       = 3.0;
    const auto withRestart = run(sc);

    ASSERT_EQUAL_(noRestart.stats.bias_restarts, 0U);
    ASSERT_GT_(withRestart.stats.bias_restarts, 0U);
    ASSERT_LT_(withRestart.maxHorizErr, 0.6 * noRestart.maxHorizErr);
}

}  // namespace

int main()
{
    try
    {
        test_off_by_default();
        test_honest_sigma();
        test_observable_with_rtk();
        test_restart_on_odometry_drift();
        std::cout << "All tests passed.\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << e.what() << "\n";
        return 1;
    }
}
