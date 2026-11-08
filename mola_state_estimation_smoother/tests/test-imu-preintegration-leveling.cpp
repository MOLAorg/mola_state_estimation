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
 * @file   test-imu-preintegration-leveling.cpp
 * @brief  IMU preintegration with a tilted {map}: T_enu_to_map is leveled from
 *         the accelerometer before its first solve.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 8, 2026
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>

#include <Eigen/Dense>  // required by mrpt's matrix accessors
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <map>
#include <string>

namespace
{
const char* PARAMS = R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    max_time_to_use_velocity_model: 2.0
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 2.0
    relative_pose_increment_sigma_lin: 0.02
    relative_pose_increment_sigma_ang: 1e-5
    min_time_difference_to_create_new_frame: 0.01
    link_first_pose_to_reference_origin_sigma: 1e-6
    estimate_geo_reference: false
    imu_preintegration:
        enabled: true
)###";

// A vehicle standing still for 5 s with the given attitude in ENU: an
// odometry source reports it at the origin of its frame (which, with the
// first pose pinned, is also {map}), and an IMU on the vehicle reads gravity.
// Returns the largest distance from the origin and the largest speed of the
// estimate over the run.
std::pair<double, double> run_static(const mrpt::poses::CPose3D& vehicleInEnu, bool resetFirst)
{
    mola::state_estimation_smoother::StateEstimationSmoother nav;
    nav.initialize(mrpt::containers::yaml::FromText(PARAMS));

    const auto cov = mrpt::math::CMatrixDouble66(mrpt::math::CMatrixDouble66::Identity() * 1e-6);
    const auto g_body =
        vehicleInEnu.inverseRotateVector(mrpt::math::TVector3D(0, 0, 9.81));  // reads "up"

    constexpr double   IMU_DT          = 0.01;
    constexpr unsigned READINGS_PER_KF = 10;
    constexpr double   DURATION        = 5.0;

    double   maxDist  = 0;
    double   maxSpeed = 0;
    unsigned solves   = 0;
    for (unsigned k = 0; k * IMU_DT < DURATION; k++)
    {
        const double t = 1.0 + k * IMU_DT;

        mrpt::obs::CObservationIMU imu;
        imu.timestamp   = mrpt::Clock::fromDouble(t);
        imu.sensorLabel = "imu";
        imu.set(mrpt::obs::IMU_X_ACC, g_body.x);
        imu.set(mrpt::obs::IMU_Y_ACC, g_body.y);
        imu.set(mrpt::obs::IMU_Z_ACC, g_body.z);
        imu.set(mrpt::obs::IMU_WX, 0.0);
        imu.set(mrpt::obs::IMU_WY, 0.0);
        imu.set(mrpt::obs::IMU_WZ, 0.0);
        nav.fuse_imu(imu);

        // A front end that levels itself from the IMU averages it first, then
        // resets the estimator and sends its first pose right away:
        if (resetFirst && k == 100)
        {
            nav.reset();
        }
        const unsigned firstPoseK = resetFirst ? 101 : 5;
        if (k < firstPoseK || (k - firstPoseK) % READINGS_PER_KF != 0)
        {
            continue;
        }
        nav.fuse_pose(
            imu.timestamp, mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D(), cov), "odom");
        const auto ns = nav.estimated_navstate(imu.timestamp, "map");
        if (!ns)
        {
            continue;
        }
        solves++;
        maxDist        = std::max(maxDist, ns->pose.mean.translation().norm());
        const auto& tw = ns->twist;
        maxSpeed = std::max(maxSpeed, std::sqrt(tw.vx * tw.vx + tw.vy * tw.vy + tw.vz * tw.vz));
    }
    ASSERT_GT_(solves, 10U);
    return {maxDist, maxSpeed};
}

// With the gravity of the preintegration factors as wrong as the attitude of
// {map}, a vehicle at rest would be estimated as falling.
void check(double yawDeg, double pitchDeg, double rollDeg, bool resetFirst = false)
{
    const auto vehicle = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        0, 0, 0, mrpt::DEG2RAD(yawDeg), mrpt::DEG2RAD(pitchDeg), mrpt::DEG2RAD(rollDeg));
    const auto [dist, speed] = run_static(vehicle, resetFirst);
    std::cout << "  max distance " << dist << " m, max speed " << speed << " m/s\n";
    ASSERT_LT_(dist, 0.05);
    ASSERT_LT_(speed, 0.05);
}
}  // namespace

void test_level() { check(0, 0, 0); }
void test_tilted() { check(40, 0, 30); }
void test_nearly_upside_down() { check(0, 0, 150); }
void test_upside_down() { check(0, 180, 0); }
void test_upside_down_after_reset() { check(0, 0, 179, true); }

int main(int argc, char** argv)
{
    const std::map<std::string, std::function<void()>> tests = {
        {"level", &test_level},
        {"tilted", &test_tilted},
        {"nearly_upside_down", &test_nearly_upside_down},
        {"upside_down", &test_upside_down},
        {"upside_down_after_reset", &test_upside_down_after_reset},
    };

    bool anyFail = false;
    for (const auto& [name, f] : tests)
    {
        try
        {
            std::cout << "[ " << name << " ] Running..." << std::endl;
            f();
            std::cout << "[ " << name << " ] OK." << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cerr << "[ " << name << " ] FAILED: " << e.what() << std::endl;
            anyFail = true;
        }
    }
    (void)argc;
    (void)argv;
    return anyFail ? 1 : 0;
}
