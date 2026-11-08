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
 * @file   test-imu-preintegration-without-imu.cpp
 * @brief  With imu_preintegration enabled but no IMU at all (odometry only), the
 *         estimates must be the same as with it disabled.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
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
const char* PARAMS = R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    max_time_to_use_velocity_model: 2.0
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 2.0
    relative_pose_increment_sigma_lin: 0.02
    relative_pose_increment_sigma_ang: 1e-5
    link_first_pose_to_reference_origin_sigma: 1e-6
    async_backend: false
    imu_preintegration:
        enabled: ENABLED
)###";

// The estimated poses for an accelerating, turning vehicle seen by odometry only:
std::vector<mrpt::poses::CPose3D> run(bool preintegration)
{
    std::string yaml = PARAMS;
    yaml.replace(yaml.find("ENABLED"), 7, preintegration ? "true" : "false");

    mola::state_estimation_smoother::StateEstimationSmoother nav;
    nav.initialize(mrpt::containers::yaml::FromText(yaml));

    auto& rng = mrpt::random::getRandomGenerator();
    rng.randomize(1234);

    std::vector<mrpt::poses::CPose3D> out;
    for (unsigned k = 0; k < 100; k++)
    {
        const double t     = 1.0 + 0.1 * k;
        const double s     = 0.05 * k * k * 0.01;  // [m] constant acceleration
        const auto   stamp = mrpt::Clock::fromDouble(t);

        mrpt::poses::CPose3DPDFGaussian pdf;
        pdf.mean = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
            s + rng.drawGaussian1D(0, 0.002), 0.1 * s * s, 0, 0.2 * s, 0, 0);
        pdf.cov.setIdentity();
        pdf.cov *= 1e-4;
        nav.fuse_pose(stamp, pdf, "odom");

        const auto ns = nav.estimated_navstate(stamp, "map");
        ASSERT_(ns.has_value());
        out.push_back(ns->pose.mean);
    }
    return out;
}
}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        const auto without = run(false);
        const auto with    = run(true);
        ASSERT_EQUAL_(without.size(), with.size());

        double maxDiff = 0;
        for (size_t i = 0; i < with.size(); i++)
        {
            maxDiff = std::max(maxDiff, (with[i].asTPose() - without[i].asTPose()).norm());
        }
        std::cout << "Largest pose difference, enabled vs disabled: " << maxDiff << "\n";
        ASSERT_LT_(maxDiff, 1e-6);

        std::cout << "All tests passed." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed: " << mrpt::exception_to_str(e) << std::endl;
        return 1;
    }
}
