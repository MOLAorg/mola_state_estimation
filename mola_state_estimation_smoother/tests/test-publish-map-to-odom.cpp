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
 * @file   test-publish-map-to-odom.cpp
 * @brief  spinOnce() emits the optional map->odom and base_link_fused updates.
 * @author Jose Luis Blanco Claraco
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>

#include <iostream>
#include <string>
#include <vector>

namespace
{
constexpr double POSE_DT    = 0.05;
constexpr double VELOCITY_X = 1.0;
constexpr size_t NUM_POSES  = 40;
const char*      ODOM_FRAME = "odom";

std::string params_yaml(bool mapToOdom, bool fused)
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
    publish_map_to_odom_tf: )###" +
        std::string(mapToOdom ? "true" : "false") +
        R"###(
    publish_fused_vehicle_tf: )###" +
        std::string(fused ? "true" : "false") + "\n";
}

using LU = mola::LocalizationSourceBase::LocalizationUpdate;

// Feeds NUM_POSES of odometry (under `sourceFrame`) and returns every
// LocalizationUpdate spinOnce() advertised. `extraYaml` appends map->odom
// routing params (map_to_odom_frame_name / map_to_odom_child_frame).
std::vector<LU> feed_and_spin_cfg(
    bool mapToOdom, bool fused, const std::string& sourceFrame, const std::string& extraYaml)
{
    mola::state_estimation_smoother::StateEstimationSmoother est;
    est.initialize(
        mrpt::containers::yaml::FromText(params_yaml(mapToOdom, fused) + extraYaml + "\n"));

    std::vector<LU> got;
    est.subscribeToLocalizationUpdates([&got](const LU& l) { got.push_back(l); });

    for (size_t i = 0; i < NUM_POSES; i++)
    {
        const auto stamp = mrpt::Clock::fromDouble(POSE_DT * static_cast<double>(i));

        // Feed odometry via fuse_pose in an odometry frame, so that odometry
        // source and its T_map_to_odom estimate exist.
        mrpt::poses::CPose3DPDFGaussian p;
        p.mean = mrpt::poses::CPose3D(VELOCITY_X * POSE_DT * static_cast<double>(i), 0, 0, 0, 0, 0);
        p.cov.setDiagonal(0.01);
        est.fuse_pose(stamp, p, sourceFrame);
    }

    // spinOnce publishes only when it has a "now" stamp and a subscriber (both
    // true here). Call a few times; each publishes the current estimate.
    for (int i = 0; i < 3; i++)
    {
        est.spinOnce();
    }
    return got;
}

std::vector<LU> feed_and_spin(bool mapToOdom, bool fused)
{
    return feed_and_spin_cfg(mapToOdom, fused, ODOM_FRAME, "");
}

size_t count_child(const std::vector<LU>& v, const std::string& child)
{
    size_t n = 0;
    for (const auto& l : v)
    {
        if (l.child_frame == child)
        {
            n++;
        }
    }
    return n;
}

void run_test()
{
    // Baseline: only the primary map->base_link update is advertised.
    {
        const auto got = feed_and_spin(false, false);
        std::cout << "baseline updates: " << got.size() << "\n";
        ASSERT_GT_(count_child(got, "base_link"), 0U);
        ASSERT_EQUAL_(count_child(got, "odom"), 0U);
        ASSERT_EQUAL_(count_child(got, "base_link_fused"), 0U);
    }

    // map->odom enabled: an extra update with child="odom", method ".../map_odom".
    {
        const auto got = feed_and_spin(true, false);
        std::cout << "map_odom updates: " << got.size() << "\n";
        ASSERT_GT_(count_child(got, "base_link"), 0U);
        ASSERT_GT_(count_child(got, "odom"), 0U);
        bool sawMethod = false;
        for (const auto& l : got)
        {
            if (l.child_frame == "odom")
            {
                ASSERT_EQUAL_(l.reference_frame, std::string("map"));
                if (l.method.find("/map_odom") != std::string::npos)
                {
                    sawMethod = true;
                }
            }
        }
        ASSERT_(sawMethod);
    }

    // base_link_fused enabled: an extra update with that child and method ".../fused".
    {
        const auto got = feed_and_spin(false, true);
        std::cout << "fused updates: " << got.size() << "\n";
        ASSERT_GT_(count_child(got, "base_link"), 0U);
        ASSERT_GT_(count_child(got, "base_link_fused"), 0U);
        bool sawMethod = false;
        for (const auto& l : got)
        {
            if (l.child_frame == "base_link_fused" && l.method.find("/fused") != std::string::npos)
            {
                sawMethod = true;
            }
        }
        ASSERT_(sawMethod);
    }

    // map_to_odom_child_frame decouples the published /tf child from the source
    // frame_id: the source is "odom_wheels" but the correction must be published
    // as map->odom to connect to an external odom->base_link.
    {
        const auto got = feed_and_spin_cfg(
            true, false, "odom_wheels",
            "    map_to_odom_frame_name: \"odom_wheels\"\n"
            "    map_to_odom_child_frame: \"odom\"");
        std::cout << "child-override updates: " << got.size() << "\n";
        ASSERT_GT_(count_child(got, "base_link"), 0U);
        // Published under the ROS odom frame, NOT the source's sensor label:
        ASSERT_GT_(count_child(got, "odom"), 0U);
        ASSERT_EQUAL_(count_child(got, "odom_wheels"), 0U);
    }

    // A map_to_odom_frame_name that names no known source must NOT silently
    // publish a bogus map->odom (which would leave {map} disconnected); the
    // primary base_link update is still advertised.
    {
        const auto got = feed_and_spin_cfg(
            true, false, "odom_wheels", "    map_to_odom_frame_name: \"does_not_exist\"");
        std::cout << "unknown-frame updates: " << got.size() << "\n";
        ASSERT_GT_(count_child(got, "base_link"), 0U);
        ASSERT_EQUAL_(count_child(got, "does_not_exist"), 0U);
        ASSERT_EQUAL_(count_child(got, "odom"), 0U);
        ASSERT_EQUAL_(count_child(got, "odom_wheels"), 0U);
    }
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
