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
 * @file   test-odometry-vote.cpp
 * @brief  The odometry vote outvotes a pose source that under-reads the
 *         distance while the wheels and the gyroscope agree.
 * @author Jose Luis Blanco Claraco
 *
 * A vehicle stands still, then drives a gentle curve. A LiDAR-odometry-like
 * pose source (an odometry frame fused with fuse_pose()) under-reads the
 * distance by 40 % for 20 s, a slow drift no single increment shows, while the
 * wheel odometry stays right. With increments as tight as the wheels', the
 * fused position follows the pose source part of the way; with the vote, its
 * sigmas are scaled while it disagrees and the estimate follows the wheels.
 * A gyroscope with a bias feeds the heading vote, which must not fire: the
 * pose source heading is right.
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random/RandomGenerators.h>

#include <cmath>
#include <iostream>
#include <optional>
#include <string>

namespace
{
const double T          = 0.1;  // [s] pose and wheel period
const size_t IMU_SUBS   = 5;  // gyroscope readings per period
const double STILL_TIME = 4.0;  // [s] standing still first (gyroscope bias)
const double DRIVE_TIME = 60.0;  // [s]
const double GT_VX      = 1.0;  // [m/s]
const double GT_WZ      = 0.02;  // [rad/s]
const double GYRO_BIAS  = 0.003;  // [rad/s]

// The pose source under-reads the distance by this factor in [SLIDE_START, SLIDE_END):
const double SLIDE_SCALE = 0.6;
const double SLIDE_START = 20.0;  // [s] of driving
const double SLIDE_END   = 40.0;

const double NOISE_XY  = 0.002;  // [m] per period
const double NOISE_ANG = 0.0002;  // [rad] per period

// Fixture check: without the vote the slide must show (8 m of distance error,
// mostly fused). With it, what remains is the detection delay: the drift until
// the disagreement exceeds the tolerance (0.8 m in a 10 m window) is already in
// the graph.
const double MIN_ERROR_WITHOUT_VOTE = 4.0;  // [m]
const double MAX_ERROR_WITH_VOTE    = 1.5;  // [m]

const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

const char* navStateParams =
    R"###(params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    link_first_pose_to_reference_origin_sigma: 1e-6
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 5.0
    max_time_to_use_velocity_model: 2.0
    sigma_random_walk_acceleration_linear: 2.0
    sigma_random_walk_acceleration_angular: 1.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.10
    odometry_min_sample_period: 0.0
    estimate_geo_reference: false
    # Pose source increments about as tight as the wheels':
    relative_pose_increment_sigma_lin: 0.005
    relative_pose_increment_sigma_ang: 0.0005
    odom_motion_model_a1: 0.05
    odom_motion_model_a4: 0.10
    # The gyroscope only feeds the vote:
    imu_angular_velocity_sigma: 0
    odometry_vote:
      enabled: VOTE
      pose_frame: "lidar_odom"
      imu_label: "imu"
)###";

double run(bool vote)
{
    auto& rng = mrpt::random::getRandomGenerator();
    rng.randomize(1234);

    std::string cfg = navStateParams;
    const auto  pos = cfg.find("VOTE");
    cfg.replace(pos, 4, vote ? "true" : "false");

    mola::state_estimation_smoother::StateEstimationSmoother stateEst;
    if (VERBOSE)
    {
        stateEst.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    stateEst.initialize(mrpt::containers::yaml::FromText(cfg));

    mrpt::poses::CPose3D gtPose     = mrpt::poses::CPose3D::Identity();
    mrpt::poses::CPose3D lioPose    = mrpt::poses::CPose3D(3.0, -2.0, 0.5, 0.4, 0, 0);
    mrpt::poses::CPose2D odomWheels = mrpt::poses::CPose2D(15.0, -8.0, 0.5);

    std::optional<mrpt::poses::CPose3D> lastEst;
    mrpt::poses::CPose3D                lastEstGt;

    const size_t n = static_cast<size_t>((STILL_TIME + DRIVE_TIME) / T);
    for (size_t i = 0; i < n; i++)
    {
        const double t      = T * static_cast<double>(i);
        const bool   moving = t >= STILL_TIME;
        const double vx     = moving ? GT_VX : 0.0;
        const double wz     = moving ? GT_WZ : 0.0;

        // Gyroscope, over the period that ends now:
        for (size_t k = 0; k < IMU_SUBS; k++)
        {
            mrpt::obs::CObservationIMU imu;
            imu.sensorLabel = "imu";
            imu.timestamp   = mrpt::Clock::fromDouble(
                  t - T + T * static_cast<double>(k + 1) / static_cast<double>(IMU_SUBS));
            imu.set(mrpt::obs::IMU_WX, rng.drawGaussian1D(0, 1e-3));
            imu.set(mrpt::obs::IMU_WY, rng.drawGaussian1D(0, 1e-3));
            imu.set(mrpt::obs::IMU_WZ, wz + GYRO_BIAS + rng.drawGaussian1D(0, 1e-3));
            stateEst.fuse_imu(imu);
        }

        if (i > 0)
        {
            const mrpt::poses::CPose3D gtDelta(vx * T, 0, 0, wz * T, 0, 0);
            gtPose = gtPose + gtDelta;

            const double drive = t - STILL_TIME;
            const double scale = (drive >= SLIDE_START && drive < SLIDE_END) ? SLIDE_SCALE : 1.0;
            const double nxy   = moving ? NOISE_XY : 0.0;
            const double nang  = moving ? NOISE_ANG : 0.0;
            lioPose            = lioPose + mrpt::poses::CPose3D(
                                               vx * T * scale + rng.drawGaussian1D(0, nxy),
                                               rng.drawGaussian1D(0, nxy), 0,
                                               wz * T + rng.drawGaussian1D(0, nang), 0, 0);
            odomWheels =
                odomWheels + mrpt::poses::CPose2D(
                                 vx * T + rng.drawGaussian1D(0, nxy), rng.drawGaussian1D(0, nxy),
                                 wz * T + rng.drawGaussian1D(0, nang));
        }

        const auto stamp = mrpt::Clock::fromDouble(t);

        mrpt::obs::CObservationOdometry obsOdo;
        obsOdo.timestamp   = stamp;
        obsOdo.sensorLabel = "wheels";
        obsOdo.odometry    = odomWheels;
        stateEst.fuse_odometry(obsOdo, "wheels");

        mrpt::poses::CPose3DPDFGaussian lio;
        lio.mean = lioPose;
        lio.cov.setDiagonal(1e-4);
        stateEst.fuse_pose(stamp, lio, "lidar_odom");

        if (const auto ns = stateEst.estimated_navstate(stamp, "map"); ns.has_value())
        {
            lastEst   = ns->pose.mean;
            lastEstGt = gtPose;
        }
    }
    ASSERT_(lastEst.has_value());
    const double err = (lastEst->translation() - lastEstGt.translation()).norm();
    std::cout << "vote " << (vote ? "on " : "off") << ": final position error " << err << " m\n";
    return err;
}
}  // namespace

int main()
{
    try
    {
        const double errOff = run(false);
        const double errOn  = run(true);
        ASSERT_GT_(errOff, MIN_ERROR_WITHOUT_VOTE);
        ASSERT_LT_(errOn, MAX_ERROR_WITH_VOTE);
        std::cout << "Test successful." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed:\n" << e.what() << std::endl;
        return 1;
    }
}
