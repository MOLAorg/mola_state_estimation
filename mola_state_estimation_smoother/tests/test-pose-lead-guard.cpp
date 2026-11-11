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
 * @file   test-pose-lead-guard.cpp
 * @brief  Parameters::pose_max_lead_time: a pose reading stamped ahead of every
 *         other input is dropped, and no correct reading is.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 11, 2026
 */

#include <mola_state_estimation_smoother/StateEstimationSmoother.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <optional>
#include <string>

namespace
{
const bool VERBOSE = mrpt::get_env<bool>("VERBOSE", false);

constexpr double      V           = 1.0;  // forward speed [m/s]
constexpr double      POSE_PERIOD = 0.1;  // [s]
constexpr double      T_END       = 7.0;  // [s]
constexpr const char* POSE_FRAME  = "lidar_odom";
constexpr const char* WHEELS      = "odom_wheels";
constexpr double      NEVER       = std::numeric_limits<double>::infinity();

using mola::state_estimation_smoother::StateEstimationSmoother;

std::string config(const double maxLead, const bool async)
{
    return std::string(R"###(
params:
    vehicle_frame_name: "base_link"
    reference_frame_name: "map"
    max_time_to_use_velocity_model: 2.0
    kinematic_model: KinematicModel::ConstantVelocity
    sliding_window_length: 3.0
    relative_pose_increment_sigma_lin: 0.02
    relative_pose_increment_sigma_ang: 0.005
    min_time_difference_to_create_new_frame: 0.01
    sigma_random_walk_acceleration_linear: 1.0
    sigma_random_walk_acceleration_angular: 10.0
    sigma_integrator_position: 0.10
    sigma_integrator_orientation: 0.10
    sigma_twist_from_consecutive_poses_linear: 1.0
    sigma_twist_from_consecutive_poses_angular: 1.0
    estimate_geo_reference: false
    pose_max_lead_time: )###") +
           std::to_string(maxLead) + "\n    async_backend: " + (async ? "true" : "false") + "\n";
}

struct Scenario
{
    double maxLead      = 0.5;  // pose_max_lead_time [s]
    bool   async        = false;
    double wheelsPeriod = 0.05;  // 0: no wheel odometry [s]
    double wheelsUntil  = NEVER;  // last wheel odometry reading [s]
    /// Stamp of the pose source reading for its pose at true time t, or none
    std::function<std::optional<double>(double)> poseStamp = [](double t) { return t; };
};

struct Result
{
    size_t drops = 0;
    /// Largest error of the frame-local prediction, asked at the true time of
    /// each pose reading before fusing it, as a front end does, once the
    /// start-up transient is over [m]
    double maxPredictionError = 0;
};

Result run(const Scenario& sc)
{
    StateEstimationSmoother est;
    if (VERBOSE)
    {
        est.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
    }
    {
        auto cfg = mrpt::containers::yaml::FromText(config(sc.maxLead, sc.async));
        est.initialize(cfg);
    }

    Result res;
    double tWheels = 0;
    for (double t = 0; t <= T_END + 1e-9; t += POSE_PERIOD)
    {
        // Wheel odometry readings up to now, as they would have arrived:
        for (; sc.wheelsPeriod > 0 && tWheels <= t + 1e-9 && tWheels <= sc.wheelsUntil;
             tWheels += sc.wheelsPeriod)
        {
            mrpt::obs::CObservationOdometry o;
            o.timestamp   = mrpt::Clock::fromDouble(tWheels);
            o.sensorLabel = WHEELS;
            o.odometry    = mrpt::poses::CPose2D(V * tWheels, 0, 0);
            est.fuse_odometry(o, WHEELS);
        }

        const auto stamp = sc.poseStamp(t);
        if (!stamp)
        {
            continue;
        }
        if (!sc.async && t > 2.0)
        {
            if (const auto pred = est.estimated_navstate(mrpt::Clock::fromDouble(t), POSE_FRAME);
                pred)
            {
                res.maxPredictionError =
                    std::max(res.maxPredictionError, std::abs(pred->pose.mean.x() - V * t));
            }
        }
        mrpt::poses::CPose3DPDFGaussian p;
        p.mean = mrpt::poses::CPose3D::FromXYZYawPitchRoll(V * t, 0, 0, 0, 0, 0);
        p.cov.setDiagonal(mrpt::square(0.01));
        est.fuse_pose(mrpt::Clock::fromDouble(*stamp), p, POSE_FRAME);
    }
    res.drops = est.pose_lead_drops();
    return res;
}

// The pose of t=3 stamped 1.5 s later, as a front end does with a scan whose
// point times are wrong: kept, it is the tail of its chain, so the correct
// readings after it are dropped as out of order, and predictions in its frame
// are extrapolated (backwards) from a pose stamped in the future.
std::optional<double> one_future_stamp(double t) { return std::abs(t - 3.0) < 1e-6 ? t + 1.5 : t; }

void test_future_stamp_dropped()
{
    Scenario sc;
    sc.poseStamp = one_future_stamp;

    const auto on = run(sc);
    std::cout << "[future stamp] guard on : drops=" << on.drops
              << " max prediction error=" << on.maxPredictionError << " m\n";
    ASSERT_EQUAL_(on.drops, 1U);
    ASSERT_LT_(on.maxPredictionError, 0.2);

    sc.maxLead     = 0;
    const auto off = run(sc);
    std::cout << "[future stamp] guard off: drops=" << off.drops
              << " max prediction error=" << off.maxPredictionError << " m\n";
    ASSERT_EQUAL_(off.drops, 0U);
    ASSERT_GT_(off.maxPredictionError, 1.0);

    // In async mode the check runs in the caller's thread, before the queue:
    sc.maxLead       = 0.5;
    sc.async         = true;
    const auto async = run(sc);
    ASSERT_EQUAL_(async.drops, 1U);
}

void test_no_drop_of_correct_readings()
{
    const auto expectNoDrops = [](const char* name, const Scenario& sc)
    {
        const auto r = run(sc);
        std::cout << "[no false drops] " << name << ": drops=" << r.drops
                  << " max prediction error=" << r.maxPredictionError << " m\n";
        ASSERT_EQUAL_(r.drops, 0U);
    };

    // The pose source alone, with a gap: nothing to compare it with.
    {
        Scenario sc;
        sc.wheelsPeriod = 0;
        sc.poseStamp    = [](double t) -> std::optional<double>
        {
            if (t > 2.0 && t < 4.0)
            {
                return std::nullopt;
            }
            return t;
        };
        expectNoDrops("alone, 2 s gap", sc);
    }
    // The other input stops: the source keeps its own pace.
    {
        Scenario sc;
        sc.wheelsUntil = 2.0;
        expectNoDrops("wheel odometry stops", sc);
    }
    // The other input stops, then the source has a gap.
    {
        Scenario sc;
        sc.wheelsUntil = 2.0;
        sc.poseStamp   = [](double t) -> std::optional<double>
        {
            if (t > 3.0 && t < 4.5)
            {
                return std::nullopt;
            }
            return t;
        };
        expectNoDrops("wheel odometry stops, then a 1.5 s gap", sc);
    }
    // The only other input is slower than the tolerance (e.g. GNSS at 1 Hz).
    {
        Scenario sc;
        sc.wheelsPeriod = 1.0;
        expectNoDrops("1 Hz other input", sc);
    }
    // The source comes back after a gap, while the other input went on.
    {
        Scenario sc;
        sc.poseStamp = [](double t) -> std::optional<double>
        {
            if (t > 3.0 && t < 4.5)
            {
                return std::nullopt;
            }
            return t;
        };
        expectNoDrops("1.5 s gap", sc);
    }
}

// From t=3 on, the stamps of the source are 2 s later than those of the other
// input (e.g. its clock was stepped): after a few readings dropped, they are
// accepted again.
void test_persistent_lead_accepted_again()
{
    Scenario sc;
    sc.poseStamp = [](double t) -> std::optional<double> { return t < 3.0 - 1e-6 ? t : t + 2.0; };
    const auto r = run(sc);
    std::cout << "[persistent lead] drops=" << r.drops << "\n";
    ASSERT_EQUAL_(r.drops, 10U);
}

}  // namespace

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;
    try
    {
        test_future_stamp_dropped();
        test_no_drop_of_correct_readings();
        test_persistent_lead_accepted_again();
        std::cout << "Test passed." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed:\n" << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
