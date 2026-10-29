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
 * @file   test-imu-attitude-yaw-bias.cpp
 * @brief  IMU attitude with a constant azimuth bias, with a fixed geo-reference
 *         (localization in a geo-referenced map) and a static vehicle with GNSS:
 *         - Without the bias state, the fused heading sigma is that of
 *           independent readings, while its error is the bias.
 *         - With it, the heading sigma stays at the bias prior while only the
 *           IMU observes the heading; once {map} poses observe it, the bias is
 *           estimated, and kept for a while after they stop.
 *         - Attitude decimation (imu_attitude_min_sample_period).
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/topography/conversions.h>

#include <cmath>
#include <iostream>

using namespace mrpt::literals;

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr double TRUE_BIAS_DEG          = 15.0;
constexpr double IMU_NOISE_DEG          = 1.0;  // white noise of each raw reading, per axis
constexpr double IMU_PERIOD             = 0.01;  // 100 Hz
constexpr double GNSS_PERIOD            = 0.2;
constexpr double MAP_POSE_PERIOD        = 0.1;
constexpr double GNSS_NOISE_XY_M        = 0.10;
constexpr double GNSS_NOISE_Z_M         = 0.15;
constexpr double MAP_POSE_SIGMA_XY      = 0.05;
constexpr double MAP_POSE_SIGMA_ANG_DEG = 0.5;

constexpr double LAT = 36.8;
constexpr double LON = -2.4;
constexpr double ALT = 50.0;

auto& rng = mrpt::random::getRandomGenerator();

const char* navStateParams = R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    max_time_to_use_velocity_model: 2.0
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 5.0
    relative_pose_increment_sigma_lin: 0.02
    relative_pose_increment_sigma_ang: 0.005
    min_time_difference_to_create_new_frame: 0.05
    sigma_random_walk_acceleration_linear: 0.1
    sigma_random_walk_acceleration_angular: 0.1
    sigma_integrator_position: 0.02
    sigma_integrator_orientation: 0.02
    estimate_geo_reference: false
    fixed_geo_reference: { latitude_deg: 36.8, longitude_deg: -2.4, altitude: 50.0 }
    imu_min_sample_period: 0.1
    imu_attitude_sigma_deg: 1.0
)###";

using Pose = mrpt::poses::CPose3D;

const Pose VEHICLE = Pose::FromXYZYawPitchRoll(3.0, -2.0, 0.1, 40.0_deg, 1.0_deg, -0.5_deg);

struct Config
{
    double yaw_bias_sigma_deg         = 0;
    double attitude_min_sample_period = 0;
};

struct Sim
{
    mola::state_estimation_smoother::StateEstimationSmoother est;
    mrpt::topography::TGeodeticCoords                        vehicleGeodetic;
    double                                                   t           = 0;
    double                                                   lastGnss    = -1e9;
    double                                                   lastMapPose = -1e9;
    double                                                   cpuTime     = 0;

    explicit Sim(const Config& c)
    {
        if (VERBOSE)
        {
            est.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
        }
        auto cfg = mrpt::containers::yaml::FromText(navStateParams);
        cfg["params"]["imu_attitude_yaw_bias_sigma_deg"] = c.yaw_bias_sigma_deg;
        cfg["params"]["imu_attitude_min_sample_period"]  = c.attitude_min_sample_period;
        est.initialize(cfg);

        const mrpt::topography::TGeodeticCoords ref(LAT, LON, ALT);
        mrpt::topography::TGeocentricCoords     geocentric;
        mrpt::topography::ENUToGeocentric(
            {VEHICLE.x(), VEHICLE.y(), VEHICLE.z()}, ref, geocentric,
            mrpt::topography::TEllipsoid::Ellipsoid_WGS84());
        mrpt::topography::geocentricToGeodetic(geocentric, vehicleGeodetic);
    }

    void fuse_imu()
    {
        mrpt::obs::CObservationIMU obs;
        obs.timestamp   = mrpt::Clock::fromDouble(t);
        obs.sensorLabel = "imu";

        // Raw reading: North-referenced (the estimator adds +90 deg), biased, noisy.
        const auto noise = Pose::FromYawPitchRoll(
            mrpt::DEG2RAD(rng.drawGaussian1D(0, IMU_NOISE_DEG)),
            mrpt::DEG2RAD(rng.drawGaussian1D(0, IMU_NOISE_DEG)),
            mrpt::DEG2RAD(rng.drawGaussian1D(0, IMU_NOISE_DEG)));
        const auto raw = Pose::FromYawPitchRoll(-90.0_deg, 0, 0) +
                         Pose::FromYawPitchRoll(mrpt::DEG2RAD(TRUE_BIAS_DEG), 0, 0) + noise +
                         Pose::FromYawPitchRoll(VEHICLE.yaw(), VEHICLE.pitch(), VEHICLE.roll());
        mrpt::math::CQuaternionDouble q;
        raw.getAsQuaternion(q);
        obs.set(mrpt::obs::IMU_ORI_QUAT_W, q.w());
        obs.set(mrpt::obs::IMU_ORI_QUAT_X, q.x());
        obs.set(mrpt::obs::IMU_ORI_QUAT_Y, q.y());
        obs.set(mrpt::obs::IMU_ORI_QUAT_Z, q.z());

        const auto localUp = VEHICLE.inverseRotateVector({0, 0, 9.81});
        obs.set(mrpt::obs::IMU_X_ACC, localUp.x + rng.drawGaussian1D(0, 0.1));
        obs.set(mrpt::obs::IMU_Y_ACC, localUp.y + rng.drawGaussian1D(0, 0.1));
        obs.set(mrpt::obs::IMU_Z_ACC, localUp.z + rng.drawGaussian1D(0, 0.1));
        obs.set(mrpt::obs::IMU_WX, rng.drawGaussian1D(0, 0.01));
        obs.set(mrpt::obs::IMU_WY, rng.drawGaussian1D(0, 0.01));
        obs.set(mrpt::obs::IMU_WZ, rng.drawGaussian1D(0, 0.01));

        est.fuse_imu(obs);
    }

    void fuse_gnss()
    {
        mrpt::obs::CObservationGPS obs;
        obs.timestamp   = mrpt::Clock::fromDouble(t);
        obs.sensorLabel = "gnss";

        constexpr double noiseDeg = GNSS_NOISE_XY_M * mrpt::RAD2DEG(1.0 / 6300e3);

        mrpt::obs::gnss::Message_NMEA_GGA gga;
        gga.fields.latitude_degrees  = vehicleGeodetic.lat + rng.drawGaussian1D(0, noiseDeg);
        gga.fields.longitude_degrees = vehicleGeodetic.lon + rng.drawGaussian1D(0, noiseDeg);
        gga.fields.altitude_meters = vehicleGeodetic.height + rng.drawGaussian1D(0, GNSS_NOISE_Z_M);
        gga.fields.fix_quality     = 4;
        obs.setMsg(gga);

        auto& cov = obs.covariance_enu.emplace();
        cov.setIdentity();
        cov(0, 0) = mrpt::square(GNSS_NOISE_XY_M);
        cov(1, 1) = mrpt::square(GNSS_NOISE_XY_M);
        cov(2, 2) = mrpt::square(GNSS_NOISE_Z_M);

        est.fuse_gnss(obs);
    }

    void fuse_map_pose()
    {
        mrpt::poses::CPose3DPDFGaussian p;
        p.mean = VEHICLE;
        p.cov.setZero();
        for (int i = 0; i < 3; i++)
        {
            p.cov(i, i)         = mrpt::square(MAP_POSE_SIGMA_XY);
            p.cov(i + 3, i + 3) = mrpt::square(mrpt::DEG2RAD(MAP_POSE_SIGMA_ANG_DEG));
        }
        est.fuse_pose(mrpt::Clock::fromDouble(t), p, "map");
    }

    /// Simulates `duration` seconds, with or without {map} poses
    void run(double duration, bool withMapPoses)
    {
        const double tEnd = t + duration;
        for (; t < tEnd; t += IMU_PERIOD)
        {
            const double t0 = mrpt::Clock::nowDouble();
            fuse_imu();
            if (t - lastGnss >= GNSS_PERIOD)
            {
                lastGnss = t;
                fuse_gnss();
            }
            if (withMapPoses && t - lastMapPose >= MAP_POSE_PERIOD)
            {
                lastMapPose = t;
                fuse_map_pose();
            }
            // As a front end would, query the estimate at its rate (this runs the solver):
            if (t - lastMapPose < 1e-9 || std::fmod(t, MAP_POSE_PERIOD) < IMU_PERIOD)
            {
                std::ignore = est.estimated_navstate(mrpt::Clock::fromDouble(t), "map");
            }
            cpuTime += mrpt::Clock::nowDouble() - t0;
        }
    }

    /// Heading error and sigma [deg]
    std::pair<double, double> heading()
    {
        const auto nv = est.estimated_navstate(mrpt::Clock::fromDouble(t), "map");
        ASSERT_(nv.has_value());
        const double err =
            mrpt::RAD2DEG(mrpt::math::angDistance(VEHICLE.yaw(), nv->pose.mean.yaw()));
        mrpt::math::CMatrixDouble66 cov;
        nv->pose.getCovariance(cov);
        const double sigma = mrpt::RAD2DEG(std::sqrt(cov(3, 3)));
        return {err, sigma};
    }
};

void print(const char* what, const std::pair<double, double>& h)
{
    std::cout << what << ": heading error " << h.first << " deg, sigma " << h.second << " deg\n";
}

void test_no_bias_state_is_overconfident()
{
    Sim s({0, 0});
    s.run(20.0, false);
    const auto h = s.heading();
    print("No bias state, IMU only", h);
    // The error is the bias, many sigmas away:
    ASSERT_GT_(std::abs(h.first), 0.8 * TRUE_BIAS_DEG);
    ASSERT_GT_(std::abs(h.first), 5 * h.second);
    ASSERT_(!s.est.estimated_imu_yaw_bias().has_value());
}

void test_bias_state()
{
    const double sigmaB = 10.0;
    Sim          s({sigmaB, 0.5});

    // 1) Only the IMU observes the heading: its sigma stays near the bias prior. Not
    //    exactly at it: fixed-lag marginalization freezes linearization points, which
    //    leaks a little information into that unobservable direction (~0.7 sigma in
    //    steady state, bounded since the bias correlation time forgets it).
    s.run(60.0, false);
    const auto h1 = s.heading();
    print("Bias state, IMU only", h1);
    ASSERT_GT_(h1.second, 0.6 * sigmaB);
    ASSERT_LT_(h1.second, 1.2 * sigmaB);
    ASSERT_LT_(std::abs(h1.first), 2.5 * h1.second);

    // 2) {map} poses observe the heading: the bias becomes known.
    s.run(20.0, true);
    const auto h2 = s.heading();
    print("Bias state, with map poses", h2);
    const auto bias2 = s.est.estimated_imu_yaw_bias();
    ASSERT_(bias2.has_value());
    std::cout << "Estimated bias: " << mrpt::RAD2DEG(bias2->first) << " deg, sigma "
              << mrpt::RAD2DEG(bias2->second) << " deg (true " << TRUE_BIAS_DEG << ")\n";
    ASSERT_LT_(std::abs(mrpt::RAD2DEG(bias2->first) - TRUE_BIAS_DEG), 2.0);
    ASSERT_LT_(mrpt::RAD2DEG(bias2->second), 2.0);
    ASSERT_LT_(std::abs(h2.first), 1.0);

    // 3) The poses stop: the bias is remembered (correlation time >> this lapse).
    s.run(10.0, false);
    const auto h3 = s.heading();
    print("Bias state, map poses lost", h3);
    ASSERT_LT_(std::abs(h3.first), 2.0);
    ASSERT_LT_(h3.second, 0.5 * sigmaB);
}

void test_attitude_decimation()
{
    // Without the bias state, the fused sigma tells how many readings were used.
    Sim all({0, 0});
    all.run(20.0, false);
    Sim decimated({0, 1.0});
    decimated.run(20.0, false);

    const auto hAll = all.heading();
    const auto hDec = decimated.heading();
    print("All attitude readings (10 Hz)", hAll);
    print("Decimated attitude readings (1 Hz)", hDec);
    // 10 times fewer readings: sqrt(10) larger sigma.
    ASSERT_GT_(hDec.second / hAll.second, 2.0);

    std::cout << "CPU time for 20 s of data: all attitude readings " << all.cpuTime
              << " s, decimated " << decimated.cpuTime << " s\n";
}

}  // namespace

int main()
{
    try
    {
        rng.randomize(1234);
        test_no_bias_state_is_overconfident();
        test_bias_state();
        test_attitude_decimation();
        std::cout << "\nSUCCESS\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "\nFAILED:\n" << e.what() << std::endl;
        return 1;
    }
}
