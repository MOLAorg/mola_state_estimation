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
 * @file   test-relocalize-not-converged-on-odometry-only.cpp
 * @brief  In relocalize mode, odometry alone must not count as a converged
 *         localization: until some source observes the map frame, it is
 *         defined as the first odometry frame by a gauge anchor, which makes
 *         the vehicle pose look certain (at the odometry origin).
 */

#include <mola_kernel/interfaces/NavStateFilter.h>
#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <iostream>
#include <tuple>

#if defined(MOLA_KERNEL_NAVSTATE_FILTER_HAS_GEO_REFERENCE)

namespace
{
const char* navStateParams =
    R"###(# Config for Parameters
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    max_time_to_use_velocity_model: 2.0
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 8.0
    relative_pose_increment_sigma_lin: 0.02
    relative_pose_increment_sigma_ang: 0.005
    min_time_difference_to_create_new_frame: 0.05
    sigma_random_walk_acceleration_linear: 0.1
    sigma_random_walk_acceleration_angular: 0.1
    sigma_integrator_position: 0.02
    sigma_integrator_orientation: 0.02
    estimate_geo_reference: false
)###";

void run_test()
{
    mola::state_estimation_smoother::StateEstimationSmoother stateEst;
    if (mrpt::get_env<bool>("VERBOSE", false))
    {
        stateEst.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    stateEst.initialize(mrpt::containers::yaml::FromText(navStateParams));

    // A geo-referenced map is loaded (relocalize mode):
    mola::Georeferencing georef;
    georef.geo_coord.lat     = 36.87;
    georef.geo_coord.lon     = -2.34;
    georef.geo_coord.height  = 100.0;
    georef.T_enu_to_map.mean = mrpt::poses::CPose3D::Identity();
    georef.T_enu_to_map.cov.setZero();
    stateEst.set_geo_reference(georef);

    // The vehicle stands still, and only wheel odometry arrives (in its own
    // frame), as before the first GNSS fix:
    mrpt::poses::CPose3DPDFGaussian odo;
    odo.cov.setDiagonal(std::vector<double>({0.01, 0.01, 0.01, 1e-4, 1e-4, 1e-4}));

    double t = 0;
    for (int i = 0; i < 20; i++, t += 0.1)
    {
        stateEst.fuse_pose(mrpt::Clock::fromDouble(t), odo, "odom_wheels");
        // Flush pending updates, as the periodic callers do:
        std::ignore = stateEst.estimated_navstate(mrpt::Clock::fromDouble(t), "map");
        std::ignore = stateEst.estimated_navstate(mrpt::Clock::fromDouble(t), "map");

        mrpt::poses::CPose3DPDFGaussian pose;
        if (stateEst.has_converged_localization(pose))
        {
            throw std::runtime_error(
                "Reported converged with odometry only, at: " + pose.mean.asString());
        }
    }
}
}  // namespace

int main()
{
    try
    {
        run_test();
        std::cout << "\nSUCCESS\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "\nFAILED:\n" << e.what() << std::endl;
        return 1;
    }
}

#else

int main()
{
    std::cout << "\nSKIPPED: installed mola_kernel lacks "
                 "NavStateFilter::set_geo_reference()/get_geo_reference()\n";
    return 0;
}
#endif
