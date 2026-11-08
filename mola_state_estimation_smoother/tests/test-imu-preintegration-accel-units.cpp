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
 * @file   test-imu-preintegration-accel-units.cpp
 * @brief  Some IMU drivers publish the acceleration in units of g. With the
 *         automatic accelerometer scale, preintegration must give the same
 *         estimates from such readings as from the same readings in m/s^2.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_gtsam_factors/gtsam_detect_version.h>
#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/random/RandomGenerators.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr double IMU_PERIOD  = 0.01;  // [s]
constexpr double POSE_PERIOD = 0.1;  // [s]
constexpr double STILL_TIME  = 2.0;  // [s]
constexpr double DRIVE_TIME  = 20.0;  // [s]
constexpr double RAMP_TIME   = 2.0;  // [s] from standing to SPEED
constexpr double SPEED       = 1.0;  // [m/s]
constexpr double CURVATURE   = 0.05;  // [1/m]
constexpr double POSE_NOISE  = 0.002;  // [m], [rad]
constexpr double G           = 9.80665;  // [m/s^2]

std::string params_yaml(const std::string& accelScale)
{
    return R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 6.0
    max_time_to_use_velocity_model: 2.0
    # A loose pose source, so the IMU shapes the estimate:
    relative_pose_increment_sigma_lin: 0.02
    relative_pose_increment_sigma_ang: 0.005
    estimate_geo_reference: false
    async_backend: false
    # Only preintegration uses the accelerometer here (the gravity factors tell
    # the units apart by themselves, reading by reading):
    imu_normalized_gravity_alignment_sigma: 0
    imu_preintegration:
        enabled: true
        accel_scale: )###" +
           accelScale + "\n";
}

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

// Vehicle pose: still, then along a circle of the given curvature.
mrpt::poses::CPose3D gtAt(double t)
{
    const double s = trackAt(t - STILL_TIME).first;
    const double r = 1.0 / CURVATURE;
    return mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        r * std::sin(CURVATURE * s), r * (1.0 - std::cos(CURVATURE * s)), 0, CURVATURE * s, 0, 0);
}

struct Reading
{
    double                          t = 0;
    mrpt::math::TVector3D           acc;  //!< [m/s^2]
    mrpt::math::TVector3D           gyr;
    bool                            hasPose = false;
    mrpt::poses::CPose3DPDFGaussian pose;
};

std::vector<Reading> simulate()
{
    auto& rng = mrpt::random::getRandomGenerator();
    rng.randomize(4321);

    const size_t         nPoseSteps = static_cast<size_t>(POSE_PERIOD / IMU_PERIOD + 0.5);
    std::vector<Reading> out;
    for (size_t i = 0; IMU_PERIOD * static_cast<double>(i) <= STILL_TIME + DRIVE_TIME; i++)
    {
        Reading r;
        r.t = IMU_PERIOD * static_cast<double>(i);

        // Level ground: tangential and centripetal acceleration plus gravity
        // (read "up"), and the turn rate:
        const double tm         = r.t - STILL_TIME;
        const double v          = trackAt(tm).second;
        const double tangential = (tm > 0 && tm < RAMP_TIME) ? SPEED / RAMP_TIME : 0.0;
        r.acc                   = {
                              tangential + rng.drawGaussian1D(0, 0.02),
                              CURVATURE * v * v + rng.drawGaussian1D(0, 0.02), G + rng.drawGaussian1D(0, 0.02)};
        r.gyr = {
            rng.drawGaussian1D(0, 0.001), rng.drawGaussian1D(0, 0.001),
            CURVATURE * v + rng.drawGaussian1D(0, 0.001)};

        if (i % nPoseSteps == 0)
        {
            r.hasPose   = true;
            r.pose.mean = gtAt(r.t) + mrpt::poses::CPose3D::FromXYZYawPitchRoll(
                                          rng.drawGaussian1D(0, POSE_NOISE),
                                          rng.drawGaussian1D(0, POSE_NOISE),
                                          rng.drawGaussian1D(0, POSE_NOISE),
                                          rng.drawGaussian1D(0, POSE_NOISE), 0, 0);
            r.pose.cov.setIdentity();
            r.pose.cov *= mrpt::square(POSE_NOISE);
        }
        out.push_back(r);
    }
    return out;
}

/// Estimated positions in {map} at the pose instants.
std::vector<mrpt::math::TPoint3D> run(
    const std::vector<Reading>& readings, double accUnit, const std::string& accelScale)
{
    mola::state_estimation_smoother::StateEstimationSmoother est;
    if (VERBOSE)
    {
        est.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    est.initialize(mrpt::containers::yaml::FromText(params_yaml(accelScale)));

    std::vector<mrpt::math::TPoint3D> positions;
    for (const auto& r : readings)
    {
        const auto stamp = mrpt::Clock::fromDouble(r.t);

        mrpt::obs::CObservationIMU imu;
        imu.timestamp   = stamp;
        imu.sensorLabel = "imu";
        imu.set(mrpt::obs::IMU_X_ACC, r.acc.x / accUnit);
        imu.set(mrpt::obs::IMU_Y_ACC, r.acc.y / accUnit);
        imu.set(mrpt::obs::IMU_Z_ACC, r.acc.z / accUnit);
        imu.set(mrpt::obs::IMU_WX, r.gyr.x);
        imu.set(mrpt::obs::IMU_WY, r.gyr.y);
        imu.set(mrpt::obs::IMU_WZ, r.gyr.z);
        est.fuse_imu(imu);

        if (!r.hasPose)
        {
            continue;
        }
        est.fuse_pose(stamp, r.pose, "odom");
        const auto ns = est.estimated_navstate(stamp, "map");
        ASSERT_(ns.has_value());
        positions.push_back(ns->pose.mean.translation());
    }
    return positions;
}

double max_distance(
    const std::vector<mrpt::math::TPoint3D>& a, const std::vector<mrpt::math::TPoint3D>& b)
{
    ASSERT_EQUAL_(a.size(), b.size());
    double d = 0;
    for (size_t i = 0; i < a.size(); i++)
    {
        d = std::max(d, (a[i] - b[i]).norm());
    }
    return d;
}

void test_imu_preintegration_accel_units()
{
#if GTSAM_USES_BOOST
    std::cout << "Skipped: IMU preintegration needs GTSAM >= 4.3\n";
    return;
#endif
    const auto readings = simulate();

    // Automatic scale (decided after the first readings):
    const auto inSI = run(readings, 1.0, "0");
    const auto inG  = run(readings, G, "0");
    // Given scale (factors attached from the first reading on):
    const auto inSIGiven = run(readings, 1.0, "1.0");
    const auto inGGiven  = run(readings, G, std::to_string(G));
    // Control: readings in units of g taken as m/s^2.
    const auto inGAsSI = run(readings, G, "1.0");

    const double dAuto  = max_distance(inSI, inG);
    const double dGiven = max_distance(inSIGiven, inGGiven);
    const double dWrong = max_distance(inSIGiven, inGAsSI);
    std::cout << "max position difference, units of g vs m/s^2: automatic scale " << dAuto
              << " m, given scale " << dGiven << " m, g taken as m/s^2 " << dWrong << " m\n";

    ASSERT_LT_(dAuto, 1e-6);
    ASSERT_LT_(dGiven, 1e-6);
    // In this scenario, the units do matter:
    ASSERT_GT_(dWrong, 0.05);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_imu_preintegration_accel_units();
        std::cout << "SUCCESS\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "FAILED:\n" << e.what() << std::endl;
        return 1;
    }
}
