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
 * @file   test-gyro-history.cpp
 * @brief  GyroHistory: rotation between two instants from the gyroscope readings
 *         (coverage, gaps, bias, label selection, which reading holds where),
 *         and the planar reduction of gyro_rotation_for_prediction().
 */

#include <mrpt/core/exceptions.h>

#include <cmath>
#include <iostream>
#include <limits>

#include "../src/GyroHistory.h"
#include "../src/extrapolation.h"

using mola::state_estimation_smoother::GyroHistory;

namespace
{
double angle_of(const Eigen::Matrix3d& R) { return Eigen::AngleAxisd(R).angle(); }

double yaw_of(const Eigen::Matrix3d& R) { return std::atan2(R(1, 0), R(0, 0)); }

void fill(GyroHistory& h, double t0, double t1, double rate, double wz)
{
    for (double t = t0; t <= t1 + 1e-9; t += 1.0 / rate)
    {
        h.add("imu", t, Eigen::Vector3d(0, 0, wz));
    }
}

void test_constant_rate()
{
    GyroHistory h(5.0);
    fill(h, 0.0, 1.0, 200, 0.5);
    const auto R = h.rotation_between(0.2, 0.3);
    ASSERT_(R.has_value());
    ASSERT_NEAR_(yaw_of(*R), 0.05, 1e-9);
}

void test_reversal()
{
    // +1 rad/s for 50 ms, then -1 rad/s for 50 ms: no net rotation, which a
    // constant rate taken at the start would miss by 0.1 rad.
    GyroHistory h(5.0);
    for (int i = 1; i <= 20; i++)
    {
        const double t = 0.005 * i;
        h.add("imu", t, Eigen::Vector3d(0, 0, t <= 0.05 ? 1.0 : -1.0));
    }
    const auto R = h.rotation_between(0.0, 0.1);
    ASSERT_(R.has_value());
    ASSERT_LT_(angle_of(*R), 1e-9);
}

void test_bias()
{
    GyroHistory h(5.0);
    fill(h, 0.0, 1.0, 200, 0.3);
    h.set_bias(Eigen::Vector3d(0, 0, 0.1));
    const auto R = h.rotation_between(0.5, 0.6);
    ASSERT_(R.has_value());
    ASSERT_NEAR_(yaw_of(*R), 0.02, 1e-9);
}

void test_coverage()
{
    GyroHistory h(5.0);
    fill(h, 0.0, 1.0, 200, 0.5);
    // After the newest reading by more than the gap, or before the oldest one:
    ASSERT_(!h.rotation_between(0.9, 1.2).has_value());
    ASSERT_(!h.rotation_between(-0.2, 0.1).has_value());
    // Backwards or empty:
    ASSERT_(!h.rotation_between(0.5, 0.5).has_value());
    ASSERT_(!h.rotation_between(0.6, 0.5).has_value());
    // A short interval between two readings holds the older one:
    const auto R = h.rotation_between(0.5001, 0.5002);
    ASSERT_(R.has_value());
    ASSERT_NEAR_(yaw_of(*R), 0.5 * 0.0001, 1e-12);

    // A gap in the middle:
    GyroHistory g(5.0);
    fill(g, 0.0, 0.3, 200, 0.5);
    fill(g, 0.5, 1.0, 200, 0.5);
    ASSERT_(!g.rotation_between(0.2, 0.6).has_value());
    ASSERT_(g.rotation_between(0.6, 0.7).has_value());
}

void test_label_selection()
{
    GyroHistory h(5.0);
    h.set_label_regex("imu.*");
    for (int i = 0; i <= 40; i++)
    {
        const double t = 0.005 * i;
        h.add("other", t, Eigen::Vector3d(0, 0, 9.0));
        h.add("imu1", t, Eigen::Vector3d(0, 0, 1.0));
        h.add("imu2", t + 0.001, Eigen::Vector3d(0, 0, 5.0));  // second match: ignored
    }
    const auto R = h.rotation_between(0.05, 0.15);
    ASSERT_(R.has_value());
    ASSERT_NEAR_(yaw_of(*R), 0.1, 1e-9);
}

void test_which_reading_holds()
{
    // A reading inside the interval holds back to the previous stamp; past the
    // last one inside, the newest reading not after that point holds. A reading
    // stamped after the interval is never used, so the result is the same
    // whether or not it has arrived:
    GyroHistory early(5.0);
    early.add("imu", 0.0, Eigen::Vector3d(0, 0, 0.0));
    early.add("imu", 0.01, Eigen::Vector3d(0, 0, 1.0));
    GyroHistory late(5.0);
    late.add("imu", 0.0, Eigen::Vector3d(0, 0, 0.0));
    late.add("imu", 0.01, Eigen::Vector3d(0, 0, 1.0));
    late.add("imu", 0.02, Eigen::Vector3d(0, 0, 2.0));
    for (const auto* h : {&early, &late})
    {
        // (0.005, 0.01] at 1 rad/s, then (0.01, 0.015] still at 1 rad/s:
        const auto R = h->rotation_between(0.005, 0.015);
        ASSERT_(R.has_value());
        ASSERT_NEAR_(yaw_of(*R), 0.01, 1e-12);
        // No reading inside: the one at 0.01 holds.
        const auto Rs = h->rotation_between(0.012, 0.016);
        ASSERT_(Rs.has_value());
        ASSERT_NEAR_(yaw_of(*Rs), 0.004, 1e-12);
    }
    // No reading inside, a changing rate around it: the older one holds.
    const auto R0 = late.rotation_between(0.002, 0.004);
    ASSERT_(R0.has_value());
    ASSERT_LT_(angle_of(*R0), 1e-12);
}

void test_non_finite_readings()
{
    constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    // One bad sample at 200 Hz: dropped, its neighbor bridges the 10 ms gap.
    GyroHistory h(5.0);
    for (int i = 0; i <= 40; i++)
    {
        const double t = 0.005 * i;
        h.add("imu", t, i == 20 ? Eigen::Vector3d(nan, 0, 9.0) : Eigen::Vector3d(0, 0, 0.5));
    }
    const auto R = h.rotation_between(0.05, 0.15);
    ASSERT_(R.has_value());
    ASSERT_NEAR_(yaw_of(*R), 0.05, 1e-9);

    // A run of bad samples longer than MAX_GAP: not covered.
    GyroHistory g(5.0);
    for (int i = 0; i <= 40; i++)
    {
        const double t   = 0.005 * i;
        const bool   bad = i >= 10 && i < 30;
        g.add("imu", t, bad ? Eigen::Vector3d(0, nan, 0) : Eigen::Vector3d(0, 0, 0.5));
    }
    ASSERT_(!g.rotation_between(0.04, 0.16).has_value());
}

void test_planar_prediction_keeps_yaw_only()
{
    GyroHistory h(5.0);
    for (int i = 0; i <= 40; i++)
    {
        h.add("imu", 0.005 * i, Eigen::Vector3d(0.4, -0.3, 0.5));
    }
    mola::state_estimation_smoother::Parameters p;
    p.imu_propagation = true;

    p.enforce_planar_motion = false;
    const auto R3 =
        mola::state_estimation_smoother::gyro_rotation_for_prediction(p, &h, 0.05, 0.15);
    ASSERT_(R3.has_value());
    ASSERT_GT_(std::abs((*R3)(2, 0)) + std::abs((*R3)(2, 1)), 1e-3);  // tilted

    p.enforce_planar_motion = true;
    const auto R2 =
        mola::state_estimation_smoother::gyro_rotation_for_prediction(p, &h, 0.05, 0.15);
    ASSERT_(R2.has_value());
    ASSERT_LT_(std::abs((*R2)(2, 0)) + std::abs((*R2)(2, 1)), 1e-12);  // no roll, no pitch
    ASSERT_NEAR_(yaw_of(*R2), yaw_of(*R3), 1e-12);
}

void test_old_readings_dropped()
{
    GyroHistory h(1.0);
    fill(h, 0.0, 3.0, 200, 0.5);
    ASSERT_(!h.rotation_between(0.5, 0.6).has_value());
    ASSERT_(h.rotation_between(2.5, 2.6).has_value());
}

}  // namespace

int main()
{
    try
    {
        test_constant_rate();
        test_reversal();
        test_bias();
        test_coverage();
        test_label_selection();
        test_which_reading_holds();
        test_non_finite_readings();
        test_planar_prediction_keeps_yaw_only();
        test_old_readings_dropped();
        std::cout << "Test successful." << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed with exception:\n" << e.what() << "\n";
        return 1;
    }
}
