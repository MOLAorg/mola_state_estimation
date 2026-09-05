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
 * @file   test-relative-pose-factors.cpp
 * @brief  Fusing a DRIFTING pose source as increments instead of as absolute
 *         poses (Parameters::relative_factors_frame_ids_re).
 *
 * A source that integrates its own increments -- visual odometry being the
 * motivating case -- has an accurate increment and an absolute pose, in its own
 * frame, that walks away from any rigid relation to {map}. Fusing it with an
 * absolute-pose factor whose covariance is the (small) per-increment one
 * asserts something false and drags the fused estimate with it. The relative
 * formulation asserts only the increment, which is what the source actually
 * knows.
 *
 * This test drives the very same measurements through both formulations and
 * checks that the relative one is the better of the two.
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random/RandomGenerators.h>

#include <iostream>

using namespace std::string_literals;
using namespace mrpt::literals;

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr size_t NUM_STEPS = 400;
constexpr double T         = 0.1;  // [s]

// The reference source: accurate, and fused as an absolute pose in {map}.
constexpr double REF_NOISE_XYZ = 0.02;  // [m] per step
constexpr double REF_NOISE_ANG = 0.002;  // [rad] per step

// The drifting source: excellent per increment, but its own frame slowly
// rotates with respect to {map} -- which is what accumulated heading drift
// looks like from the outside, and what visual odometry actually does. A
// zero-mean random walk would NOT make the point: the estimator can average
// that away across the window. Systematic drift is what an absolute-pose
// factor cannot represent, because it asserts ONE rigid frame transform.
constexpr double DRIFT_NOISE_XYZ = 0.005;  // [m] per step
constexpr double DRIFT_NOISE_ANG = 0.0005;  // [rad] per step

std::string navStateParams(const std::string& relativeFramesRe)
{
    return R"###(# Config for Parameters
params:
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

    estimate_geo_reference: false
    relative_factors_frame_ids_re: ")###" +
           relativeFramesRe + R"###("
)###";
}

/** Runs the identical measurement stream through the estimator, with the
 *  drifting source fused either relatively or absolutely, and returns the RMS
 *  position error of the fused estimate against ground truth. */
double run(bool drifterIsRelative, double driftFrameRate)
{
    mola::state_estimation_smoother::StateEstimationSmoother stateEst;
    if (VERBOSE)
    {
        stateEst.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    stateEst.initialize(mrpt::containers::yaml::FromText(
        navStateParams(drifterIsRelative ? "drifting_odom"s : ""s)));

    // Same seed for both runs: the two formulations must see the same data.
    auto& rng = mrpt::random::getRandomGenerator();
    rng.randomize(1234);

    mrpt::poses::CPose3D gtPose  = mrpt::poses::CPose3D::Identity();
    mrpt::poses::CPose3D refOdom = mrpt::poses::CPose3D::Identity();
    // Start the drifting source somewhere else entirely: its frame transform is
    // for the estimator to work out, in both formulations.
    mrpt::poses::CPose3D driftOdom =
        mrpt::poses::CPose3D::FromXYZYawPitchRoll(7.0, -3.0, 0.5, 30.0_deg, 0.0_deg, 0.0_deg);

    const auto gtDelta = mrpt::poses::CPose3D(1.0 * T, 0, 0, 0.2 * T, 0, 0);

    double               sumSqErr = 0;
    double               sumSqSrc = 0;
    size_t               nEval    = 0;
    mrpt::poses::CPose3D lastEst;
    mrpt::poses::CPose3D lastGt;
    mrpt::poses::CPose3D lastDrift;
    bool                 havePrev = false;

    for (size_t i = 0; i < NUM_STEPS; i++)
    {
        const auto stamp = mrpt::Clock::fromDouble(T * static_cast<double>(i));
        if (i > 0)
        {
            gtPose = gtPose + gtDelta;
        }

        // Reference source: noisy increments, fused as an absolute pose.
        auto refDelta = gtDelta;
        refDelta.x_incr(rng.drawGaussian1D(0, REF_NOISE_XYZ));
        refDelta.y_incr(rng.drawGaussian1D(0, REF_NOISE_XYZ));
        refDelta.setYawPitchRoll(
            refDelta.yaw() + rng.drawGaussian1D(0, REF_NOISE_ANG), refDelta.pitch(),
            refDelta.roll());
        refOdom = refOdom + refDelta;

        mrpt::poses::CPose3DPDFGaussian refPdf;
        refPdf.mean = refOdom;
        refPdf.cov.setZero();
        for (int k = 0; k < 3; k++)
        {
            refPdf.cov(k, k)         = mrpt::square(REF_NOISE_XYZ);
            refPdf.cov(k + 3, k + 3) = mrpt::square(REF_NOISE_ANG);
        }

        // Drifting source: finer increments than the reference...
        auto driftDelta = gtDelta;
        driftDelta.x_incr(rng.drawGaussian1D(0, DRIFT_NOISE_XYZ));
        driftDelta.y_incr(rng.drawGaussian1D(0, DRIFT_NOISE_XYZ));
        driftDelta.setYawPitchRoll(
            driftDelta.yaw() + rng.drawGaussian1D(0, DRIFT_NOISE_ANG), driftDelta.pitch(),
            driftDelta.roll());
        driftOdom = driftOdom + driftDelta;

        // ...reported in a frame that keeps sliding. Per increment this is a
        // 5 mm perturbation, inside the sigma quoted below; across the
        // estimator's 5 s window it is 0.25 m, fifty times that sigma, and no
        // single rigid transform can absorb it. That gap -- small per
        // increment, unbounded once integrated -- IS drift, and it is what an
        // absolute-pose factor cannot represent.
        const mrpt::poses::CPose3D driftFrame = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
            driftFrameRate * T * static_cast<double>(i), 0, 0, 0, 0, 0);

        mrpt::poses::CPose3DPDFGaussian driftPdf;
        driftPdf.mean = driftFrame + driftOdom;
        driftPdf.cov.setZero();
        for (int k = 0; k < 3; k++)
        {
            driftPdf.cov(k, k)         = mrpt::square(DRIFT_NOISE_XYZ);
            driftPdf.cov(k + 3, k + 3) = mrpt::square(DRIFT_NOISE_ANG);
        }

        stateEst.fuse_pose(stamp, refPdf, "reference_odom");
        stateEst.fuse_pose(stamp, driftPdf, "drifting_odom");

        // Skip the transient while the frame transforms are still being solved.
        if (i < NUM_STEPS / 4)
        {
            continue;
        }
        // Both sources' frames float freely with respect to {map}; what is
        // observable, and what a front end consumes, is the MOTION. Score the
        // estimate on its increment over the last 10 steps against GT's.
        if ((i % 10) == 0)
        {
            const auto st = stateEst.estimated_navstate(stamp, "map");
            if (!st.has_value())
            {
                continue;
            }
            if (havePrev)
            {
                const auto   dEst = st->pose.mean - lastEst;
                const auto   dGt  = gtPose - lastGt;
                const auto   dSrc = driftOdom - lastDrift;
                const double e    = (dEst.translation() - dGt.translation()).norm();
                sumSqSrc += (dSrc.translation() - dGt.translation()).sqrNorm();
                sumSqErr += e * e;
                ++nEval;
                if (VERBOSE)
                {
                    std::cout << "i=" << i << " err=" << e << " dEst=" << dEst.asString()
                              << " dGt=" << dGt.asString() << "\n";
                }
            }
            lastEst   = st->pose.mean;
            lastGt    = gtPose;
            lastDrift = driftOdom;
            havePrev  = true;
        }
    }
    ASSERT_(nEval > 10);
    std::cout << "   (drifting source's OWN motion error = "
              << std::sqrt(sumSqSrc / static_cast<double>(nEval)) << " m)\n";
    return std::sqrt(sumSqErr / static_cast<double>(nEval));
}
}  // namespace

int main()
{
    try
    {
        // The same source, with and without a slowly sliding frame of its own.
        // 0.05 m/s perturbs one increment by 5 mm -- inside the sigma the source
        // declares -- while displacing its absolute pose by 0.25 m across the
        // estimator's 5 s window, fifty times that sigma.
        const double absNoDrift = run(/*relative=*/false, 0.0);
        const double absDrift   = run(/*relative=*/false, 0.05);
        const double relNoDrift = run(/*relative=*/true, 0.0);
        const double relDrift   = run(/*relative=*/true, 0.05);

        std::cout << "[relative-pose-factors] ABSOLUTE: no drift = " << absNoDrift
                  << " m, with drift = " << absDrift << " m\n";
        std::cout << "[relative-pose-factors] RELATIVE: no drift = " << relNoDrift
                  << " m, with drift = " << relDrift << " m\n";

        // What the formulation buys is INVARIANCE to the source's absolute
        // drift, not a lower floor: it asserts only the increment, which the
        // drift barely touches. The absolute formulation asserts one rigid
        // frame transform, which the drift makes false, so it degrades.
        ASSERTMSG_(
            absDrift > 1.5 * absNoDrift,
            mrpt::format(
                "The absolute formulation should degrade under the source's drift "
                "(%.4f -> %.4f m)",
                absNoDrift, absDrift));

        ASSERTMSG_(
            std::abs(relDrift - relNoDrift) < 0.2 * relNoDrift,
            mrpt::format(
                "The relative formulation should be insensitive to the source's absolute "
                "drift (%.4f -> %.4f m)",
                relNoDrift, relDrift));

        // ...and it must still track, not merely be indifferent.
        ASSERT_LT_(relDrift, 0.15);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << e.what() << "\n";
        return 1;
    }
    std::cout << "Test passed.\n";
    return 0;
}
