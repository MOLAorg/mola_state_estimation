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
 * @file   test-motion-prior-from-simple.cpp
 * @brief  motion_prior_from_simple: predictions in the pose source's frame are
 *         those of a standalone simple estimator fed the same readings, while
 *         the graph keeps answering in the reference frame.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 5, 2026
 */

#include <mola_state_estimation_simple/StateEstimationSimple.h>
#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/Lie/SE.h>

#include <iostream>

namespace
{
const char* smootherParams = R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 5.0
    max_time_to_use_velocity_model: 0.75
    relative_pose_increment_sigma_lin: 0.02
    relative_pose_increment_sigma_ang: 0.005
    async_backend: false
    motion_prior_from_simple: true
    motion_prior_simple:
        max_time_to_use_velocity_model: 0.75
        sigma_relative_pose_linear: 0.5
)###";

const char* simpleParams = R"###(
params:
    max_time_to_use_velocity_model: 0.75
    sigma_relative_pose_linear: 0.5
)###";

void run_test()
{
    mola::state_estimation_smoother::StateEstimationSmoother smoother;
    smoother.initialize(mrpt::containers::yaml::FromText(smootherParams));
    mola::state_estimation_simple::StateEstimationSimple simple;
    simple.initialize(mrpt::containers::yaml::FromText(simpleParams));

    const double T     = 0.1;  // [s] pose period
    const double TIMU  = 0.01;  // [s]
    const double V     = 1.0;  // [m/s]
    const double W     = 0.3;  // [rad/s]
    auto         odom  = mrpt::poses::CPose3D::Identity();
    size_t       nSame = 0;

    for (size_t i = 0; i < 100; i++)
    {
        const double t = T * static_cast<double>(i);
        for (size_t k = 0; k < 10; k++)
        {
            auto imu         = mrpt::obs::CObservationIMU::Create();
            imu->timestamp   = mrpt::Clock::fromDouble(t + TIMU * static_cast<double>(k));
            imu->sensorLabel = "imu";
            imu->set(mrpt::obs::IMU_WZ, W);
            imu->set(mrpt::obs::IMU_WX, 0.0);
            imu->set(mrpt::obs::IMU_WY, 0.0);
            smoother.onNewObservation(imu);
            simple.onNewObservation(imu);
        }
        if (i > 0)
        {
            odom = odom + mrpt::poses::CPose3D(V * T, 0, 0, W * T, 0, 0);
        }
        mrpt::poses::CPose3DPDFGaussian pdf;
        pdf.mean = odom;
        pdf.cov.setIdentity();
        pdf.cov *= 1e-4;
        const auto stamp = mrpt::Clock::fromDouble(t);
        smoother.fuse_pose(stamp, pdf, "odom");
        simple.fuse_pose(stamp, pdf, "odom");

        const auto query = mrpt::Clock::fromDouble(t + 0.05);
        const auto a     = smoother.estimated_navstate(query, "odom");
        const auto b     = simple.estimated_navstate(query, "odom");
        ASSERT_EQUAL_(a.has_value(), b.has_value());
        if (a && b)
        {
            const double d = mrpt::poses::Lie::SE<3>::log(a->pose.mean - b->pose.mean).norm();
            ASSERT_LT_(d, 1e-9);
            nSame++;
        }
    }
    std::cout << "identical predictions: " << nSame << "\n";
    ASSERT_GT_(nSame, 90U);

    // The graph still answers in its own reference frame:
    ASSERT_(smoother.estimated_navstate(mrpt::Clock::fromDouble(9.9), "map").has_value());
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
