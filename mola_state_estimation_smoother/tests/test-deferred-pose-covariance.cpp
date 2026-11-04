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
 * @file   test-deferred-pose-covariance.cpp
 * @brief  fuse_pose_deferred(): same result as fuse_pose() once the covariance
 *         is known, and, with the async backend, the caller never waits for it.
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/poses/CPose3D.h>

#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace std::string_literals;

#if defined(MOLA_KERNEL_NAVSTATE_FILTER_HAS_FUSE_POSE_DEFERRED)
namespace
{
constexpr double POSE_DT    = 0.05;  // [s]
constexpr double VELOCITY_X = 1.0;  // [m/s]
constexpr size_t NUM_POSES  = 40;
const double     EXPECTED_X = VELOCITY_X * POSE_DT * static_cast<double>(NUM_POSES - 1);
const char*      ODOM_FRAME = "odom";

std::string params_yaml(bool async)
{
    return
        R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    link_first_pose_to_reference_origin_sigma: 1e-6
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 10.0
    relative_pose_increment_sigma_lin: 0.02
    relative_pose_increment_sigma_ang: 0.005
    relative_pose_increment_from_source_covariance: true
    max_time_to_use_velocity_model: 5.0
    estimate_geo_reference: false
    async_backend: )###" +
        (async ? "true"s : "false"s) + "\n";
}

mrpt::poses::CPose3D mean_at(size_t i)
{
    return {VELOCITY_X * POSE_DT * static_cast<double>(i), 0, 0, 0, 0, 0};
}

mrpt::math::CMatrixDouble66 cov_at(size_t i)
{
    mrpt::math::CMatrixDouble66 c;
    c.setDiagonal(1e-4 * (1.0 + 0.1 * static_cast<double>(i % 3)));
    return c;
}

mrpt::Clock::time_point stamp_at(size_t i)
{
    return mrpt::Clock::fromDouble(POSE_DT * static_cast<double>(i));
}

mrpt::poses::CPose3D run_sync(bool deferred)
{
    mola::state_estimation_smoother::StateEstimationSmoother est;
    est.initialize(mrpt::containers::yaml::FromText(params_yaml(false)));
    for (size_t i = 0; i < NUM_POSES; i++)
    {
        if (deferred)
        {
            std::promise<mrpt::math::CMatrixDouble66> p;
            p.set_value(cov_at(i));
            est.fuse_pose_deferred(stamp_at(i), mean_at(i), p.get_future().share(), ODOM_FRAME);
        }
        else
        {
            est.fuse_pose(
                stamp_at(i), mrpt::poses::CPose3DPDFGaussian(mean_at(i), cov_at(i)), ODOM_FRAME);
        }
    }
    const auto nav = est.estimated_navstate(stamp_at(NUM_POSES - 1), "map");
    ASSERT_(nav.has_value());
    return nav->pose.mean;
}

void test_sync_equivalence()
{
    const auto a = run_sync(false);
    const auto b = run_sync(true);
    std::cout << "sync: fuse_pose " << a << " fuse_pose_deferred " << b << "\n";
    ASSERTMSG_(a == b, "fuse_pose_deferred() must equal fuse_pose() in sync mode");
}

// The covariances arrive late, from another thread: neither fuse_pose_deferred()
// nor estimated_navstate() may wait for them, and the estimate must still
// converge once they are all known.
void test_async_never_waits()
{
    mola::state_estimation_smoother::StateEstimationSmoother est;
    est.initialize(mrpt::containers::yaml::FromText(params_yaml(true)));

    std::vector<std::promise<mrpt::math::CMatrixDouble66>> promises(NUM_POSES);
    double                                                 maxCallMs = 0;
    for (size_t i = 0; i < NUM_POSES; i++)
    {
        const auto t0 = std::chrono::steady_clock::now();
        est.fuse_pose_deferred(
            stamp_at(i), mean_at(i), promises[i].get_future().share(), ODOM_FRAME);
        (void)est.estimated_navstate(stamp_at(i), ODOM_FRAME);
        const double ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0)
                .count();
        maxCallMs = std::max(maxCallMs, ms);
    }

    // The producer resolves them all, 0.3 s later:
    std::thread producer(
        [&]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            for (size_t i = 0; i < NUM_POSES; i++)
            {
                promises[i].set_value(cov_at(i));
            }
        });

    double mapX = std::nan("");
    for (int iter = 0; iter < 800; iter++)  // up to ~4 s
    {
        const auto nav = est.estimated_navstate(stamp_at(NUM_POSES - 1), "map");
        if (nav.has_value())
        {
            mapX = nav->pose.mean.x();
            if (std::abs(mapX - EXPECTED_X) < 0.05)
            {
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    producer.join();
    std::cout << "async: slowest call while covariances were pending " << maxCallMs << " ms; map x "
              << mapX << " (expected " << EXPECTED_X << ")\n";
    ASSERT_LT_(maxCallMs, 100.0);
    ASSERT_LT_(std::abs(mapX - EXPECTED_X), 0.05);
}

}  // namespace
#endif

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
#if defined(MOLA_KERNEL_NAVSTATE_FILTER_HAS_FUSE_POSE_DEFERRED)
        test_sync_equivalence();
        test_async_never_waits();
#endif
        std::cout << "✅ SUCCESS\n";
        return 0;
    }
    catch (std::exception& e)
    {
        std::cerr << "❌ FAILED: " << e.what() << std::endl;
        return 1;
    }
}
