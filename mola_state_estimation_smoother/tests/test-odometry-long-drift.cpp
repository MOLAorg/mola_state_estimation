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
 * @file   test-odometry-long-drift.cpp
 * @brief  Wheel odometry that dead-reckons hundreds of metres away must not drag
 *         the fused trajectory with it.
 * @author Jose Luis Blanco Claraco
 * @date   Aug 21, 2026
 *
 * Measured when this was written, and worth knowing before reading a result
 * from it: **both odometry formulations pass this, and by two orders of
 * magnitude.** With 600 m of true travel, a 4% scale error and periodic 4x slip
 * bursts -- 209 m of accumulated odometry error in total -- the fused
 * trajectory ends 0.02 m from the truth whether the increments enter as
 * absolute poses in {odom_i} or as relative constraints between keyframes.
 *
 * That is not the fixture being weak; it is the graph being right, and the
 * reason is `T_map_to_odom_i`. It is a free variable with only a weak prior,
 * it is re-optimized on every update, and it is never marginalized (its key
 * timestamp is bumped to the newest observation each time). So it SLIDES to
 * absorb accumulated odometry drift -- which is exactly what a REP-105
 * map->odom correction is for. The absolute factor's mean is therefore not
 * "wrong by 200 m" in the graph's own terms: the frame it is measured against
 * moved with it. The inference is forced by the numbers above -- had that frame
 * stayed put, factors asserting a pose 200 m away with ~0.2 m sigma could not
 * have left the estimate at 0.02 m.
 *
 * So this file does NOT discriminate between the two formulations, and must not
 * be cited as evidence for either. What it does guard is real and was worth
 * pinning down: that unbounded dead-reckoning drift, smooth or bursty, does not
 * leak into the fused trajectory when an accurate pose source is present.
 *
 * The case that DOES separate them is not reachable from here: on
 * BotanicGarden the relative formulation is better on 5 of 7 sequences, and
 * that is a closed loop -- the estimator's output becomes the front end's ICP
 * prior, which shapes the next pose it is fed. A standalone estimator test
 * feeds poses that do not depend on what the estimator said, so it cannot
 * reproduce that at all.
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random/RandomGenerators.h>

#include <cmath>
#include <iostream>
#include <optional>

using namespace std::string_literals;
using namespace mrpt::literals;

namespace
{
// A long drive: 1200 steps of 0.1 s at 5 m/s is 600 m of travel. Long enough
// that a few per cent of scale error becomes tens of metres, which is the
// regime this test exists for and the one a short fixture cannot reach.
const size_t NUM_POSES = 1200;
const double T         = 0.1;  // [s] sensor period
const double GT_VX     = 5.0;  // [m/s]
const double GT_WZ     = 0.05;  // [rad/s], a gentle curve: a straight line
                            // leaves yaw unobservable and is a weaker test

// The wheel odometry is 4% long. Nothing about that is unusual -- it is what an
// uncalibrated wheel radius, a tyre pressure change or soft ground produce --
// and it is SYSTEMATIC, not zero-mean noise, which is the whole point.
const double WHEEL_SCALE_ERROR = 1.04;
const double WHEEL_NOISE_XY    = 0.005;  // [m] per step, on top of the bias
const double WHEEL_NOISE_PHI   = 0.0005;  // [rad] per step

// Slip: every SLIP_PERIOD steps the wheels spin for SLIP_LENGTH steps and
// report SLIP_FACTOR times the real motion. This is the part a slow scale error
// does not reproduce -- error that arrives FAST relative to the sliding window,
// which is what a skid-steer platform on loose ground actually does.
const size_t SLIP_PERIOD = 50;
const size_t SLIP_LENGTH = 5;
const double SLIP_FACTOR = 4.0;

// The other source: an accurate pose in the reference frame, i.e. what a lidar
// front end feeds in through fuse_pose(). This is the arrangement every MOLA-LO
// run with wheel odometry actually has.
const double LIDAR_NOISE_XYZ = 0.02;  // [m]
const double LIDAR_NOISE_ANG = 0.002;  // [rad]

// With an accurate pose source available, the fused trajectory should stay with
// it. Sized well clear of the lidar noise floor, and roughly two orders of
// magnitude below the odometry's own final error, so the test says something
// about whether the bias leaks in rather than about tuning.
const double MAXIMUM_FINAL_ERROR = 1.0;  // [m]

// Sanity bound on the fixture itself: if the simulated odometry does not
// actually run away, the test is not testing what it claims to.
const double MINIMUM_ODOMETRY_DRIFT = 15.0;  // [m]

const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

auto& rng = mrpt::random::getRandomGenerator();

const char* navStateParams =
    R"###(# Config for Parameters
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"

    # No GNSS here, so the map frame is defined by where the robot starts.
    link_first_pose_to_reference_origin_sigma: 1e-6

    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 5.0
    max_time_to_use_velocity_model: 2.0

    sigma_random_walk_acceleration_linear: 2.0
    sigma_random_walk_acceleration_angular: 1.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.10

    # Keep every reading: the decimation is orthogonal to what is under test.
    odometry_min_sample_period: 0.0

    estimate_geo_reference: false
)###";

void run_test()
{
    mola::state_estimation_smoother::StateEstimationSmoother stateEst;
    if (VERBOSE)
    {
        stateEst.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    stateEst.initialize(mrpt::containers::yaml::FromText(navStateParams));

    mrpt::poses::CPose3D gtPose = mrpt::poses::CPose3D::Identity();

    std::optional<mrpt::poses::CPose3D> lastEst;
    mrpt::poses::CPose3D                lastEstGt;
    size_t                              lastEstIndex = 0;
    size_t                              numEstimates = 0;

    // The odometry frame starts somewhere arbitrary, as a real one does: what
    // ties it to {map} is the estimator, not a shared origin.
    mrpt::poses::CPose2D odomWheels(15.0, -8.0, 30.0_deg);

    const auto gtDelta = mrpt::poses::CPose3D(GT_VX * T, 0, 0, GT_WZ * T, 0, 0);

    double gtTravelled   = 0;
    double odomTravelled = 0;

    for (size_t i = 0; i < NUM_POSES; i++)
    {
        const auto stamp = mrpt::Clock::fromDouble(T * static_cast<double>(i));

        if (i > 0)
        {
            gtPose = gtPose + gtDelta;
            gtTravelled += GT_VX * T;

            // Wheel odometry: the same motion, over-reported by a fixed scale
            // factor, plus a little white noise.
            const bool   slipping = (i % SLIP_PERIOD) < SLIP_LENGTH;
            const double slip     = slipping ? SLIP_FACTOR : 1.0;

            mrpt::poses::CPose2D deltaWheels(
                gtDelta.x() * WHEEL_SCALE_ERROR * slip + rng.drawGaussian1D(0, WHEEL_NOISE_XY),
                rng.drawGaussian1D(0, WHEEL_NOISE_XY),
                gtDelta.yaw() + rng.drawGaussian1D(0, WHEEL_NOISE_PHI));
            odomWheels = odomWheels + deltaWheels;
            odomTravelled += deltaWheels.norm();
        }

        // Accurate pose in {map}, as a lidar front end would provide.
        mrpt::poses::CPose3DPDFGaussian lidarPdf;
        lidarPdf.mean =
            gtPose +
            mrpt::poses::CPose3D(
                rng.drawGaussian1D(0, LIDAR_NOISE_XYZ), rng.drawGaussian1D(0, LIDAR_NOISE_XYZ),
                rng.drawGaussian1D(0, LIDAR_NOISE_XYZ), rng.drawGaussian1D(0, LIDAR_NOISE_ANG),
                rng.drawGaussian1D(0, LIDAR_NOISE_ANG), rng.drawGaussian1D(0, LIDAR_NOISE_ANG));
        lidarPdf.cov.setDiagonal(mrpt::square(LIDAR_NOISE_XYZ));
        for (int k = 3; k < 6; k++)
        {
            lidarPdf.cov(k, k) = mrpt::square(LIDAR_NOISE_ANG);
        }

        mrpt::obs::CObservationOdometry obsOdo;
        obsOdo.timestamp   = stamp;
        obsOdo.sensorLabel = "wheels";
        obsOdo.odometry    = odomWheels;

        stateEst.fuse_odometry(obsOdo, "wheels");
        stateEst.fuse_pose(stamp, lidarPdf, "map");

        // Track the newest usable estimate as we go. Queried here rather than
        // once at the end because a query can legitimately return nothing (an
        // under-constrained window), and "the estimator went quiet" is itself a
        // failure mode worth counting rather than crashing on.
        if (const auto ns = stateEst.estimated_navstate(stamp, "map"); ns.has_value())
        {
            lastEst      = ns->pose.mean;
            lastEstGt    = gtPose;
            lastEstIndex = i;
            numEstimates++;
        }
    }

    // How far the raw odometry ended up from the truth. Its own frame offset is
    // a rigid transform the estimator can and does absorb, so the quantity that
    // matters is the DISTANCE TRAVELLED, which a scale error corrupts and no
    // rigid transform can repair.
    const double odomDrift = std::abs(odomTravelled - gtTravelled);

    std::cout << "ground truth travelled: " << gtTravelled << " m\n"
              << "wheel odometry travelled: " << odomTravelled << " m\n"
              << "odometry drift: " << odomDrift << " m\n";

    // The fixture has to actually exercise the regime it claims to.
    ASSERT_GT_(odomDrift, MINIMUM_ODOMETRY_DRIFT);

    std::cout << "usable estimates: " << numEstimates << " of " << NUM_POSES << ", newest at step "
              << lastEstIndex << "\n";

    // An estimator that simply stops answering has not passed this test.
    ASSERT_(lastEst.has_value());
    ASSERT_GT_(numEstimates, NUM_POSES / 2);
    ASSERT_GT_(lastEstIndex, (NUM_POSES * 9) / 10);

    const double finalError = (lastEst->translation() - lastEstGt.translation()).norm();

    std::cout << "final fused error: " << finalError << " m (limit " << MAXIMUM_FINAL_ERROR
              << " m)\n";

    // The point of the whole exercise: an odometry source that has dead-reckoned
    // tens of metres away must not drag the fused trajectory with it. It cannot,
    // if its increments enter as RELATIVE constraints -- each one states a
    // short-baseline motion whose error is small and zero-mean. It necessarily
    // does if they enter as an absolute pose in the odometry frame, because that
    // pose carries the whole accumulated bias in its MEAN, and a mean that is
    // wrong by tens of metres is not made harmless by widening its covariance.
    ASSERT_LT_(finalError, MAXIMUM_FINAL_ERROR);
}
}  // namespace

int main()
{
    try
    {
        run_test();
        std::cout << "Test successful." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed:\n" << e.what() << std::endl;
        return 1;
    }
}
