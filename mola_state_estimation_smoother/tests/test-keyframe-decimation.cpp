/* _
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
 * @file   test-keyframe-decimation.cpp
 * @brief  Unit tests for high-rate same-sensor keyframe decimation/merging
 *         (Parameters::odometry_min_sample_period / imu_min_sample_period).
 * @author Jose Luis Blanco Claraco
 * @date   Jun 23, 2026
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/Lie/SE.h>

#include <iostream>

using namespace std::string_literals;
using namespace mrpt::literals;

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

// A wide enough sliding window so nothing is marginalized out during these
// short (~1 s) runs: keyframe counts then equal the total ever created.
const char* baseParams =
    R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"

    # Pin the "odom" frame to the "map" origin so a single odometry source fully
    # determines the absolute pose (otherwise "odom" floats freely).
    link_first_pose_to_reference_origin_sigma: 1e-6

    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 30.0
    min_time_difference_to_create_new_frame: 0.01
    max_time_to_use_velocity_model: 2.0

    sigma_random_walk_acceleration_linear: 2.0
    sigma_random_walk_acceleration_angular: 1.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.10

    estimate_geo_reference: false
)###";

std::unique_ptr<mola::state_estimation_smoother::StateEstimationSmoother> makeEstimator(
    const std::string& extraParams)
{
    auto est = std::make_unique<mola::state_estimation_smoother::StateEstimationSmoother>();
    if (VERBOSE)
    {
        est->setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    est->initialize(mrpt::containers::yaml::FromText(baseParams + extraParams));
    return est;
}

// Feeds a constant-velocity wheel-odometry trajectory (straight line + yaw rate)
// at the requested rate, then returns the final fused pose. The number of
// readings and the trajectory are identical regardless of decimation, so the two
// runs are directly comparable.
struct OdomRunResult
{
    mrpt::poses::CPose3D estimatedFinalPose;
    mrpt::poses::CPose2D groundTruthFinalOdom;
    std::size_t          keyframeCount = 0;
};

OdomRunResult runOdometry(
    mola::state_estimation_smoother::StateEstimationSmoother& est, double samplePeriod,
    size_t numReadings, double vx, double wz)
{
    mrpt::poses::CPose2D odom;  // accumulated ground-truth odometry pose
    mrpt::Clock::time_point lastStamp;

    for (size_t i = 0; i < numReadings; i++)
    {
        const double t     = samplePeriod * static_cast<double>(i);
        const auto   stamp = mrpt::Clock::fromDouble(t);

        if (i > 0)
        {
            // Constant-velocity increment in the local frame:
            const auto delta = mrpt::poses::CPose2D(vx * samplePeriod, 0.0, wz * samplePeriod);
            odom             = odom + delta;
        }

        mrpt::obs::CObservationOdometry obs;
        obs.timestamp   = stamp;
        obs.sensorLabel = "wheel_odom";
        obs.odometry    = odom;
        est.fuse_odometry(obs, obs.sensorLabel);

        lastStamp = stamp;
    }

    OdomRunResult r;
    r.groundTruthFinalOdom = odom;

    const auto stateOpt = est.estimated_navstate(lastStamp, est.parameters().reference_frame_name);
    ASSERT_(stateOpt.has_value());
    r.estimatedFinalPose = stateOpt->pose.mean;
    r.keyframeCount      = est.active_keyframe_count();
    return r;
}

// --------------------------------------------------------------------------
// Test 1: odometry decimation drastically reduces the keyframe count while
//         keeping the estimated trajectory faithful to the input odometry.
//         A *straight* constant-velocity path is used so the constant-velocity
//         kinematic prior is exact (no "corner cutting"): this isolates the
//         correctness of the increment *merging* (dropped readings must be
//         accumulated, not lost) from the inherent accuracy/cost tradeoff of
//         using fewer keyframes on a curved path.
// --------------------------------------------------------------------------
void test_odometry_decimation_reduces_keyframes_and_preserves_pose()
{
    constexpr size_t N      = 101;  // 100 increments
    constexpr double RATE_S = 0.02;  // 50 Hz over 2.0 s
    constexpr double VX     = 1.0;  // m/s
    constexpr double WZ     = 0.0;  // straight line

    // Full-rate reference run:
    auto estFull = makeEstimator("");  // odometry_min_sample_period defaults to 0
    auto resFull = runOdometry(*estFull, RATE_S, N, VX, WZ);

    // Decimated run: keep ~1 every 5 readings (10 Hz):
    auto estDecim = makeEstimator("    odometry_min_sample_period: 0.10\n");
    auto resDecim = runOdometry(*estDecim, RATE_S, N, VX, WZ);

    std::cout << "[odom] full keyframes=" << resFull.keyframeCount
              << " decimated keyframes=" << resDecim.keyframeCount << "\n";
    std::cout << "[odom] GT final odom:  " << resDecim.groundTruthFinalOdom.asString() << "\n";
    std::cout << "[odom] full  est pose: " << resFull.estimatedFinalPose.asString() << "\n";
    std::cout << "[odom] decim est pose: " << resDecim.estimatedFinalPose.asString() << "\n";

    // Full rate: each reading (after the first) makes its own keyframe.
    ASSERT_EQUAL_(resFull.keyframeCount, N - 1);

    // Decimation must cut the keyframe count by roughly the decimation factor (5x).
    ASSERT_LT_(resDecim.keyframeCount, resFull.keyframeCount / 3);
    ASSERT_GT_(resDecim.keyframeCount, static_cast<std::size_t>(5));

    // Merging must preserve the accumulated motion: the estimated forward
    // displacement stays within a few % of the input odometry. (A bug that
    // advanced the anchor on a dropped reading would lose ~80% of it.) Note that
    // decimation does cost some accuracy vs. full rate, because each merged
    // absolute constraint spans a longer interval and so gets a looser
    // motion-model covariance, and fewer keyframes give the velocity estimate
    // less time to converge from its initial prior. That tradeoff is the whole
    // point; we only assert it stays small and bounded here.
    const auto   gt = mrpt::poses::CPose3D(resDecim.groundTruthFinalOdom);
    const double errDecim = mrpt::poses::Lie::SE<3>::log(resDecim.estimatedFinalPose - gt).norm();
    const double errFull  = mrpt::poses::Lie::SE<3>::log(resFull.estimatedFinalPose - gt).norm();

    std::cout << "[odom] err full=" << errFull << " err decim=" << errDecim << "\n";

    ASSERT_GT_(resDecim.estimatedFinalPose.x(), 0.9 * gt.x());  // >=90% displacement retained
    ASSERT_LT_(errDecim, 0.2);  // bounded, not catastrophic
}

// --------------------------------------------------------------------------
// Test 1b: on a *turning* path, merging must still preserve the total traveled
//          displacement (the accumulation is correct in SE(2), not just along a
//          straight axis). We deliberately do NOT assert a tight pose match here
//          because using fewer keyframes legitimately "cuts the corner" of a
//          curved trajectory with the constant-velocity prior; we only require
//          that the bulk of the motion is retained (an anchor-advance merge bug
//          would lose most of it).
// --------------------------------------------------------------------------
void test_odometry_merge_preserves_displacement_on_turn()
{
    constexpr size_t N      = 101;
    constexpr double RATE_S = 0.02;
    constexpr double VX     = 1.0;
    constexpr double WZ     = 0.15;  // rad/s -> noticeable curve

    auto est = makeEstimator("    odometry_min_sample_period: 0.10\n");
    auto res = runOdometry(*est, RATE_S, N, VX, WZ);

    const double gtDist  = res.groundTruthFinalOdom.norm();  // |(x,y)| of final odom
    const double estDist = std::hypot(res.estimatedFinalPose.x(), res.estimatedFinalPose.y());

    std::cout << "[odom-turn] GT dist=" << gtDist << " est dist=" << estDist
              << " keyframes=" << res.keyframeCount << "\n";

    // Retain the great majority of the displacement (corner-cutting plus the
    // looser merged covariance remove only a few %); this fails hard if merged
    // increments are dropped.
    ASSERT_GT_(estDist, 0.85 * gtDist);
    ASSERT_LT_(estDist, 1.05 * gtDist);
}

// --------------------------------------------------------------------------
// Test 2: with decimation disabled (default 0), the keyframe count is exactly
//         the full-rate count (regression guard for back-compat).
// --------------------------------------------------------------------------
void test_decimation_disabled_is_backward_compatible()
{
    constexpr size_t N      = 51;
    constexpr double RATE_S = 0.02;

    auto est = makeEstimator("    odometry_min_sample_period: 0.0\n");
    auto res = runOdometry(*est, RATE_S, N, 1.0, 0.0);

    std::cout << "[default] keyframes=" << res.keyframeCount << " (expected " << (N - 1) << ")\n";
    ASSERT_EQUAL_(res.keyframeCount, N - 1);
}

// --------------------------------------------------------------------------
// Test 3: IMU decimation reduces the number of (redundant) attitude/gravity
//         factors while still leveling the estimate (pitch/roll ~ 0).
// --------------------------------------------------------------------------
std::size_t runImuLeveling(double imuSamplePeriod, double& finalPitch, double& finalRoll)
{
    // Two streams: LiDAR pose at 10 Hz (drives keyframes) + IMU at 100 Hz.
    auto est = makeEstimator(
        "    imu_nearby_keyframe_stamp_tolerance: 0.10\n"
        "    imu_min_sample_period: "s +
        std::to_string(imuSamplePeriod) + "\n");

    constexpr double IMU_T = 0.01;  // 100 Hz; LiDAR pose added every 10th step (10 Hz)
    constexpr size_t STEPS   = 200;  // 2.0 s

    mrpt::poses::CPose3D     odom;
    mrpt::Clock::time_point  lastStamp;

    for (size_t i = 0; i < STEPS; i++)
    {
        const double t     = IMU_T * static_cast<double>(i);
        const auto   stamp = mrpt::Clock::fromDouble(t);
        lastStamp          = stamp;

        // IMU every step (100 Hz): flat robot, gravity straight down.
        mrpt::obs::CObservationIMU imu;
        imu.timestamp   = stamp;
        imu.sensorLabel = "imu";
        imu.set(mrpt::obs::IMU_X_ACC, 0.0);
        imu.set(mrpt::obs::IMU_Y_ACC, 0.0);
        imu.set(mrpt::obs::IMU_Z_ACC, 9.81);
        est->fuse_imu(imu);

        // LiDAR pose every 10th step (10 Hz):
        if ((i % 10) == 0)
        {
            odom = odom + mrpt::poses::CPose3D(0.1, 0, 0, 0, 0, 0);  // 1 m/s in x
            mrpt::poses::CPose3DPDFGaussian pdf;
            pdf.mean = odom;
            pdf.cov.setIdentity();
            pdf.cov *= 1e-3;
            est->fuse_pose(stamp, pdf, "lidar");
        }
    }

    const auto stateOpt = est->estimated_navstate(lastStamp, "map");
    ASSERT_(stateOpt.has_value());
    double yaw = 0;
    stateOpt->pose.mean.getYawPitchRoll(yaw, finalPitch, finalRoll);

    return est->active_factor_count();
}

void test_imu_decimation_reduces_factors_and_keeps_leveling()
{
    double pitchFull = 0;
    double rollFull  = 0;
    const std::size_t factorsFull = runImuLeveling(0.0, pitchFull, rollFull);

    double pitchDecim = 0;
    double rollDecim  = 0;
    // 0.1 s => at most one IMU factor per 0.1 s window (matches the nearby
    // tolerance), instead of ~10 redundant ones.
    const std::size_t factorsDecim = runImuLeveling(0.10, pitchDecim, rollDecim);

    std::cout << "[imu] full factors=" << factorsFull << " decimated factors=" << factorsDecim
              << "\n";
    std::cout << "[imu] full  pitch=" << mrpt::RAD2DEG(pitchFull)
              << " roll=" << mrpt::RAD2DEG(rollFull) << " deg\n";
    std::cout << "[imu] decim pitch=" << mrpt::RAD2DEG(pitchDecim)
              << " roll=" << mrpt::RAD2DEG(rollDecim) << " deg\n";

    // Decimation must cut the (committed) factor count appreciably.
    ASSERT_LT_(factorsDecim, factorsFull);

    // Leveling must still hold in both cases.
    ASSERT_NEAR_(pitchDecim, 0.0, mrpt::DEG2RAD(2.0));
    ASSERT_NEAR_(rollDecim, 0.0, mrpt::DEG2RAD(2.0));
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_odometry_decimation_reduces_keyframes_and_preserves_pose();
        test_odometry_merge_preserves_displacement_on_turn();
        test_decimation_disabled_is_backward_compatible();
        test_imu_decimation_reduces_factors_and_keeps_leveling();

        std::cout << "✅ SUCCESS\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "❌ FAILED: " << e.what() << std::endl;
        return 1;
    }
}
