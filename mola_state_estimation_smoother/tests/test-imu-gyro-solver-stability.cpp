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
 * @file   test-imu-gyro-solver-stability.cpp
 * @brief  Unit test: fusing raw IMU gyro readings (imu_angular_velocity_sigma>0)
 *         must not destabilize the solver. Replays a realistic LIO session: an
 *         IMU-only keyframe chain built while the front end levels itself, then
 *         lagged scan-matching poses spliced back into it at drifting stamps, so
 *         keyframe pairs land just past the merge threshold (~10 ms apart). The
 *         constant-angular-velocity factor is tightest exactly there, and its
 *         residual (R_i*w_i - R_j*w_j) can only be relieved by rotating the
 *         poses, so an over-trusted gyro prior used to blow up the whole window.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/random/RandomGenerators.h>

#include <cmath>
#include <iostream>
#include <string>

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr double IMU_PERIOD    = 1.0 / 400;  // raw IMU rate [s] (decimated inside)
constexpr double LIDAR_PERIOD  = 0.1;  // scan-matching pose rate [s]
constexpr double LIDAR_T0      = 3.0;  // first ICP pose: the front end levels itself first [s]
constexpr double LIDAR_OFFSET  = 0.037;  // pose stamps deliberately off the IMU keyframe grid
constexpr double LIDAR_DRIFT   = 0.001;  // per-scan stamp drift: sweeps every keyframe gap size
constexpr double LIDAR_LAG     = 0.09;  // scan-matching latency: the pose is fused this late [s]
constexpr double DURATION      = 20.0;  // total simulated time [s]
constexpr double VX_TRUE       = 1.0;  // [m/s]
constexpr double WZ_TRUE       = 1.0;  // [rad/s]
constexpr double GRAVITY       = 9.81;
constexpr double GYRO_NOISE    = 0.1;  // raw gyro per-sample noise [rad/s]
constexpr double POSE_NOISE_XYZ = 0.01;  // scan-matching position noise [m]
constexpr double POSE_NOISE_ANG = 0.005;  // scan-matching orientation noise [rad]
// The front end starts its map frame at the identity, wherever the vehicle
// happened to be standing, so the map is NOT gravity-levelled: every gravity
// factor must be absorbed by the shared T_enu_to_map variable.
constexpr double MAP_TILT_DEG  = 6.5;

auto& rng = mrpt::random::getRandomGenerator();

// Mirrors the shipped state-estimation-smoother.yaml defaults (the values a
// real mola_lidar_odometry run uses), except for the synchronous backend, so
// the test is deterministic.
std::string make_config(double gyroSigma)
{
    return std::string(R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 6.0
    max_time_to_use_velocity_model: 0.75
    min_time_difference_to_create_new_frame: 0.01
    imu_nearby_keyframe_stamp_tolerance: 0.10
    imu_min_sample_period: 0.1
    sigma_random_walk_acceleration_linear: 0.5
    sigma_random_walk_acceleration_angular: 1.0
    sigma_integrator_position: 0.1
    sigma_integrator_orientation: 1.0
    sigma_twist_from_consecutive_poses_linear: 1.0
    sigma_twist_from_consecutive_poses_angular: 1.0
    initial_twist_sigma_lin: 20.0
    initial_twist_sigma_ang: 3.0
    imu_normalized_gravity_alignment_sigma: 0.4
    additional_isam2_update_steps: 3
    estimate_geo_reference: false
    async_backend: false
    imu_angular_velocity_sigma: )###") +
           std::to_string(gyroSigma) + "\n";
}

struct Result
{
    size_t solverFailures = 0;
    double maxPoseError   = 0;
    double maxPoseErrorT  = 0;
};

// Replays the session described in the file header. Returns the solver-failure
// count and the worst short-term pose prediction error seen.
Result run_session(double gyroSigma, uint32_t seed)
{
    Result res;
    rng.randomize(seed);

    mola::state_estimation_smoother::StateEstimationSmoother est;
    est.initialize(mrpt::containers::yaml::FromText(make_config(gyroSigma)));

    est.logRegisterCallback(
        [&res](
            std::string_view msg, const mrpt::system::VerbosityLevel level, std::string_view,
            const mrpt::Clock::time_point)
        {
            if (level >= mrpt::system::LVL_ERROR &&
                msg.find("GTSAM update failed") != std::string_view::npos)
            {
                res.solverFailures++;
            }
        });

    // Rotation from the (gravity-levelled) world to the front end's map frame:
    const auto mapFromLevel =
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(0, 0, 0, 0, mrpt::DEG2RAD(-MAP_TILT_DEG), 0);

    // The IMU is not aligned with the vehicle frame (as in a real setup):
    const auto imuOnVehicle =
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(0.1, 0.0, 0.2, mrpt::DEG2RAD(30.0), 0, 0);

    // Ground truth: stationary while the front end levels itself, then constant
    // forward speed plus a gentle turn.
    const auto gtPoseAt = [](double t)
    {
        const double tm = std::max(0.0, t - LIDAR_T0);
        mrpt::math::CVectorFixedDouble<6> d;
        d.setZero();
        d[0] = VX_TRUE * tm;
        d[5] = WZ_TRUE * tm;
        return mrpt::poses::CPose3D(mrpt::poses::Lie::SE<3>::exp(d));
    };

    const auto fuseImuAt = [&](double t)
    {
        const auto gtPose = gtPoseAt(t);

        mrpt::obs::CObservationIMU obs;
        obs.timestamp   = mrpt::Clock::fromDouble(t);
        obs.sensorLabel = "imu";
        obs.sensorPose  = imuOnVehicle;

        // Accelerometer: gravity as seen in the IMU frame. The poses live in the
        // tilted map frame, so gravity must be expressed there too.
        const auto imuPose = mapFromLevel + gtPose + imuOnVehicle;
        const auto g_world = mrpt::math::TVector3D(0, 0, GRAVITY);
        const auto g_imu   = imuPose.inverseRotateVector(g_world);
        obs.set(mrpt::obs::IMU_X_ACC, g_imu.x + rng.drawGaussian1D(0, 0.05));
        obs.set(mrpt::obs::IMU_Y_ACC, g_imu.y + rng.drawGaussian1D(0, 0.05));
        obs.set(mrpt::obs::IMU_Z_ACC, g_imu.z + rng.drawGaussian1D(0, 0.05));

        // Gyro: the vehicle rate expressed in the IMU frame.
        const auto w_vehicle = mrpt::math::TVector3D(0, 0, t < LIDAR_T0 ? 0.0 : WZ_TRUE);
        const auto w_imu     = imuOnVehicle.inverseRotateVector(w_vehicle);
        // Per-sample gyro noise well above imu_angular_velocity_sigma, as on a
        // vibrating real platform: consecutive readings genuinely disagree.
        obs.set(mrpt::obs::IMU_WX, w_imu.x + rng.drawGaussian1D(0, GYRO_NOISE));
        obs.set(mrpt::obs::IMU_WY, w_imu.y + rng.drawGaussian1D(0, GYRO_NOISE));
        obs.set(mrpt::obs::IMU_WZ, w_imu.z + rng.drawGaussian1D(0, GYRO_NOISE));

        est.fuse_imu(obs);
    };

    const auto fusePoseAt = [&](double t, double sigma)
    {
        mrpt::poses::CPose3DPDFGaussian p;
        p.mean = gtPoseAt(t);
        p.mean.x(p.mean.x() + rng.drawGaussian1D(0, POSE_NOISE_XYZ));
        p.mean.y(p.mean.y() + rng.drawGaussian1D(0, POSE_NOISE_XYZ));
        p.mean.z(p.mean.z() + rng.drawGaussian1D(0, POSE_NOISE_XYZ));
        p.mean.setYawPitchRoll(
            p.mean.yaw() + rng.drawGaussian1D(0, POSE_NOISE_ANG),
            p.mean.pitch() + rng.drawGaussian1D(0, POSE_NOISE_ANG),
            p.mean.roll() + rng.drawGaussian1D(0, POSE_NOISE_ANG));
        p.cov.setIdentity();
        p.cov *= mrpt::square(sigma);
        est.fuse_pose(mrpt::Clock::fromDouble(t), p, "map");
    };

    double nextLidar   = LIDAR_T0;
    double lidarOffset = LIDAR_OFFSET;

    for (size_t i = 0; IMU_PERIOD * static_cast<double>(i) <= DURATION; i++)
    {
        const double t = IMU_PERIOD * static_cast<double>(i);

        // NOTE: no pose whatsoever is fused before LIDAR_T0. A LIO front end
        // levels itself against gravity while stationary before it emits any
        // pose, so the estimator runs IMU-only for the first seconds: position,
        // yaw and linear velocity are then unobservable, held by nothing but
        // the first keyframe's weak priors and the kinematic chain.

        // Scan matching takes about one scan period, so each pose reaches the
        // estimator with a timestamp well BEHIND the IMU readings already
        // fused: it is spliced back into the middle of the existing keyframe
        // chain, not appended at its end.
        while (nextLidar + lidarOffset + LIDAR_LAG <= t)
        {
            fusePoseAt(nextLidar + lidarOffset, 0.02);
            nextLidar += LIDAR_PERIOD;
            lidarOffset += LIDAR_DRIFT;
        }

        fuseImuAt(t);

        // Score at 100 Hz, not at the full IMU rate: enough to catch a diverging
        // prediction, and it keeps the test's runtime down.
        if (i % 4 != 0)
        {
            continue;
        }

        // Only score once pose observations have been flowing for a while: before
        // that the estimator is legitimately dead-reckoning with no velocity
        // information at all.
        if (t < LIDAR_T0 + 1.5)
        {
            continue;
        }

        const auto st = est.estimated_navstate(mrpt::Clock::fromDouble(t), "map");
        if (st.has_value())
        {
            const auto err = (st->pose.mean - gtPoseAt(t)).norm();
            if (err > res.maxPoseError)
            {
                res.maxPoseError  = err;
                res.maxPoseErrorT = t;
            }
        }
    }

    if (VERBOSE)
    {
        std::cout << "[gyroSigma=" << gyroSigma << "] solverFailures=" << res.solverFailures
                  << " maxPoseError=" << res.maxPoseError << " at t=" << res.maxPoseErrorT << "\n";
    }
    return res;
}

// Enabling the gyro prior must never destabilize the solver: no iSAM2 failure
// (each one forces a full smoother reset and metre-level pose errors), and
// accuracy comparable to the gyro-less baseline.
void test_solver_is_stable_with_gyro_prior()
{
    // Several fixed seeds: the conflict this guards against is data-dependent,
    // so one realization is not enough to make the test reliable.
    for (uint32_t seed = 1; seed <= 5; seed++)
    {
        const auto withoutGyro = run_session(0.0, seed);
        const auto withGyro    = run_session(0.05, seed);

        ASSERT_EQUAL_(withoutGyro.solverFailures, 0U);
        ASSERT_EQUAL_(withGyro.solverFailures, 0U);

        ASSERT_LT_(withoutGyro.maxPoseError, 0.5);
        ASSERT_LT_(withGyro.maxPoseError, 0.5);
    }
}

}  // namespace

int main()
{
    try
    {
        test_solver_is_stable_with_gyro_prior();
        std::cout << "Test successful." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed with exception:\n" << e.what() << "\n";
        return 1;
    }
}
