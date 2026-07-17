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
 * @file   test-async-navstate.cpp
 * @brief  Async backend: eventual correctness, sync-equivalence, and a
 *         concurrency stress (no deadlock/crash while querying under load).
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/get_env.h>
#include <mrpt/poses/CPose3D.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

using namespace std::string_literals;

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr double POSE_DT    = 0.05;  // [s]  (20 Hz)
constexpr double VELOCITY_X = 1.0;  // [m/s]
constexpr size_t NUM_POSES  = 60;
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
    max_time_to_use_velocity_model: 5.0
    sigma_random_walk_acceleration_linear: 2.0
    sigma_random_walk_acceleration_angular: 1.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.10
    estimate_geo_reference: false
    async_backend: )###" +
        (async ? "true"s : "false"s) + "\n";
}

mrpt::poses::CPose3DPDFGaussian pose_at(size_t i)
{
    mrpt::poses::CPose3DPDFGaussian p;
    p.mean = mrpt::poses::CPose3D(VELOCITY_X * POSE_DT * static_cast<double>(i), 0, 0, 0, 0, 0);
    p.cov.setDiagonal(0.001);
    return p;
}

/// Feeds NUM_POSES into a sync instance and returns the final map-frame x.
double run_sync()
{
    mola::state_estimation_smoother::StateEstimationSmoother est;
    est.initialize(mrpt::containers::yaml::FromText(params_yaml(false)));

    mrpt::Clock::time_point last;
    for (size_t i = 0; i < NUM_POSES; i++)
    {
        const auto stamp = mrpt::Clock::fromDouble(POSE_DT * static_cast<double>(i));
        last             = stamp;
        est.fuse_pose(stamp, pose_at(i), ODOM_FRAME);
    }
    const auto nav = est.estimated_navstate(last, "map");
    ASSERT_(nav.has_value());
    return nav->pose.mean.x();
}

/// Feeds NUM_POSES into an async instance, waits for the backend to catch up,
/// and returns the final x in both the {map} and {odom} frames.
std::pair<double, double> run_async()
{
    mola::state_estimation_smoother::StateEstimationSmoother est;
    if (VERBOSE)
    {
        est.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    est.initialize(mrpt::containers::yaml::FromText(params_yaml(true)));

    mrpt::Clock::time_point last;
    for (size_t i = 0; i < NUM_POSES; i++)
    {
        const auto stamp = mrpt::Clock::fromDouble(POSE_DT * static_cast<double>(i));
        last             = stamp;
        est.fuse_pose(stamp, pose_at(i), ODOM_FRAME);
    }

    // Poll until the backend has solved enough that the map-frame estimate
    // reaches the expected endpoint (or time out).
    double mapX  = std::nan("");
    double odomX = std::nan("");
    for (int iter = 0; iter < 800; iter++)  // up to ~4 s
    {
        const auto navMap = est.estimated_navstate(last, "map");
        if (navMap.has_value())
        {
            mapX = navMap->pose.mean.x();
            if (std::abs(mapX - EXPECTED_X) < 0.05)
            {
                const auto navOdom = est.estimated_navstate(last, ODOM_FRAME);
                if (navOdom.has_value())
                {
                    odomX = navOdom->pose.mean.x();
                    break;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return {mapX, odomX};
}

void test_equivalence()
{
    const double syncX                 = run_sync();
    const auto [asyncMapX, asyncOdomX] = run_async();

    std::cout << "expected x:      " << EXPECTED_X << "\n";
    std::cout << "sync map x:      " << syncX << "\n";
    std::cout << "async map x:     " << asyncMapX << "\n";
    std::cout << "async odom x:    " << asyncOdomX << "\n";

    ASSERT_(std::isfinite(asyncMapX));
    ASSERT_(std::isfinite(asyncOdomX));
    // Async eventually reaches the correct fused pose...
    ASSERT_LT_(std::abs(asyncMapX - EXPECTED_X), 0.05);
    // ...and the frame-local {odom} path agrees too...
    ASSERT_LT_(std::abs(asyncOdomX - EXPECTED_X), 0.10);
    // ...and matches the synchronous estimator on the same input.
    ASSERT_LT_(std::abs(asyncMapX - syncX), 0.05);
}

// Hammer fuse_*() from several threads while querying, to shake out deadlocks
// and data races in the async ingest path.
void test_concurrency()
{
    mola::state_estimation_smoother::StateEstimationSmoother est;
    est.initialize(mrpt::containers::yaml::FromText(params_yaml(true)));

    std::atomic<bool>   stop{false};
    std::atomic<size_t> queries{0};
    std::atomic<double> lastStampSec{0.0};

    std::thread producer(
        [&]
        {
            for (size_t i = 0; i < 400; i++)
            {
                const double tsec = POSE_DT * static_cast<double>(i);
                est.fuse_pose(mrpt::Clock::fromDouble(tsec), pose_at(i), ODOM_FRAME);
                lastStampSec.store(tsec);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            stop = true;
        });

    std::thread consumer(
        [&]
        {
            while (!stop.load())
            {
                const double tsec = lastStampSec.load();
                if (tsec > 0.0)
                {
                    if (est.estimated_navstate(mrpt::Clock::fromDouble(tsec), "map").has_value())
                    {
                        queries++;
                    }
                }
                std::this_thread::sleep_for(std::chrono::microseconds(200));
            }
        });

    producer.join();
    consumer.join();

    std::cout << "concurrency: served " << queries.load() << " queries, no deadlock\n";
    ASSERT_GT_(queries.load(), static_cast<size_t>(0));
}

}  // namespace

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        test_equivalence();
        test_concurrency();
        std::cout << "✅ SUCCESS\n";
        return 0;
    }
    catch (std::exception& e)
    {
        std::cerr << "❌ FAILED: " << e.what() << std::endl;
        return 1;
    }
}
