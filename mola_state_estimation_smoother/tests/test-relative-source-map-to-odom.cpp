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
 * @file   test-relative-source-map-to-odom.cpp
 * @brief  The estimated map->odom of a source fused as relative increments
 *         must follow that source's drift, not stay at the initial alignment.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <cmath>
#include <iostream>
#include <string>

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr double DURATION   = 10.0;  // [s]
constexpr double VELOCITY_X = 1.0;  // [m/s]
constexpr double ODOM_DT    = 0.02;  // [s]  (50 Hz)
constexpr double POSE_DT    = 0.10;  // [s]  (10 Hz)
constexpr double ODOM_SCALE = 1.10;  // the wheels overestimate by 10%
constexpr double POSE_SIGMA = 0.01;  // [m]

const char* PARAMS_YAML = R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    link_first_pose_to_reference_origin_sigma: 1e-6
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 3.0
    max_time_to_use_velocity_model: 2.0
    sigma_random_walk_acceleration_linear: 2.0
    sigma_random_walk_acceleration_angular: 1.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.10
    estimate_geo_reference: false
    odometry_min_sample_period: 0.0
)###";

void run_test()
{
    mola::state_estimation_smoother::StateEstimationSmoother est;
    if (VERBOSE)
    {
        est.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    est.initialize(mrpt::containers::yaml::FromText(PARAMS_YAML));

    // Interleave both streams in timestamp order.
    const auto nOdom = static_cast<size_t>(std::lround(DURATION / ODOM_DT));
    const auto step  = static_cast<size_t>(std::lround(POSE_DT / ODOM_DT));

    mrpt::Clock::time_point lastStamp;
    mrpt::poses::CPose2D    lastOdom;
    for (size_t i = 0; i <= nOdom; i++)
    {
        const double t = ODOM_DT * static_cast<double>(i);
        lastStamp      = mrpt::Clock::fromDouble(t);

        if (i % step == 0)
        {
            mrpt::poses::CPose3DPDFGaussian pdf;
            pdf.mean =
                mrpt::poses::CPose3D::FromXYZYawPitchRoll(VELOCITY_X * t, 0.0, 0.0, 0.0, 0.0, 0.0);
            for (int k = 0; k < 3; k++)
            {
                pdf.cov(k, k) = POSE_SIGMA * POSE_SIGMA;
            }
            for (int k = 3; k < 6; k++)
            {
                pdf.cov(k, k) = mrpt::square(0.005);
            }
            est.fuse_pose(lastStamp, pdf, "map");
            // As a front end does once per scan; this also runs the solver
            // regularly, so the window slides past the odometry anchor.
            (void)est.estimated_navstate(lastStamp, "map");
        }

        mrpt::obs::CObservationOdometry odom;
        odom.timestamp = lastStamp;
        odom.odometry  = mrpt::poses::CPose2D(ODOM_SCALE * VELOCITY_X * t, 0.0, 0.0);
        lastOdom       = odom.odometry;
        est.fuse_odometry(odom, "wheels_odom");
    }

    const auto nav = est.estimated_navstate(lastStamp, "map");
    ASSERT_(nav.has_value());
    const auto mapToOdom = est.estimated_T_map_to_odometry_frame("wheels_odom");
    ASSERT_(mapToOdom.has_value());

    // The accumulated drift is 1 m. Composed with the latest odometry, the
    // estimated map->odom must land on the fused pose; a map->odom stuck at the
    // initial alignment would miss it by that whole drift.
    const auto   viaOdom = mapToOdom->mean + mrpt::poses::CPose3D(lastOdom);
    const double err     = viaOdom.distanceTo(nav->pose.mean);

    std::cout << "fused x: " << nav->pose.mean.x() << ", map->odom x: " << mapToOdom->mean.x()
              << ", map->odom (+) odom x: " << viaOdom.x() << ", error: " << err << " m\n";

    ASSERT_LT_(err, 0.05);
    ASSERT_LT_(mapToOdom->mean.x(), -0.5);
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        run_test();
        std::cout << "✅ SUCCESS\n";
        return 0;
    }
    catch (std::exception& e)
    {
        std::cerr << "❌ FAILED: " << e.what() << std::endl;
        return 1;
    }
}
