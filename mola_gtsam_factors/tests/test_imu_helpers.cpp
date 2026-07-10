/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this package
 alone or in combination with the complete SLAM system.
*/

#include <mola_gtsam_factors/imu_helpers.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
void expect(bool cond, const std::string& msg)
{
    if (!cond)
    {
        throw std::runtime_error("Test assertion failed: " + msg);
    }
}

constexpr double kDeg2Rad = M_PI / 180.0;

void test_imu_accel_looks_like_gravity()
{
    using mola::factors::imu_accel_looks_like_gravity;

    expect(imu_accel_looks_like_gravity({0, 0, 9.81}), "raw m/s^2 gravity should be accepted");
    expect(imu_accel_looks_like_gravity({0, 0, 1.0}), "normalized gravity should be accepted");
    expect(
        imu_accel_looks_like_gravity({5.0, 5.0, 6.9}),
        "off-axis but plausible-norm gravity should be accepted");

    expect(
        !imu_accel_looks_like_gravity({0, 0, 0.05}), "too-small norm should not look like gravity");
    expect(
        !imu_accel_looks_like_gravity({0, 0, 20.0}), "too-large norm should not look like gravity");
    expect(!imu_accel_looks_like_gravity({0, 0, 0}), "zero vector should not look like gravity");
}

void test_imu_quaternion_looks_valid()
{
    using mola::factors::imu_quaternion_looks_valid;

    expect(imu_quaternion_looks_valid(1, 0, 0, 0), "identity quaternion should be valid");

    // An arbitrary normalized quaternion:
    const double n = std::sqrt(0.5 * 0.5 + 0.5 * 0.5 + 0.5 * 0.5 + 0.5 * 0.5);
    expect(
        imu_quaternion_looks_valid(0.5 / n, 0.5 / n, 0.5 / n, 0.5 / n),
        "arbitrary normalized quaternion should be valid");

    expect(!imu_quaternion_looks_valid(2, 0, 0, 0), "non-normalized quaternion should be invalid");
    expect(!imu_quaternion_looks_valid(0, 0, 0, 0), "zero quaternion should be invalid");

    const double nan = std::numeric_limits<double>::quiet_NaN();
    expect(!imu_quaternion_looks_valid(nan, 0, 0, 0), "NaN quaternion should be invalid");
}

void test_imu_apply_enu_azimuth_correction()
{
    using mola::factors::imu_apply_enu_azimuth_correction;

    // Definition under test: result = Rz(90deg + offset) * rawAttitude.
    for (const double offsetDeg : {0.0, 10.0, -30.0, 179.0})
    {
        for (const auto& raw :
             {gtsam::Rot3::Identity(),
              gtsam::Rot3::Ypr(kDeg2Rad * 20, kDeg2Rad * 5, kDeg2Rad * -8)})
        {
            const auto got      = imu_apply_enu_azimuth_correction(raw, offsetDeg);
            const auto expected = gtsam::Rot3::Rz((90.0 + offsetDeg) * kDeg2Rad) * raw;

            expect(
                got.equals(expected, 1e-9),
                "imu_apply_enu_azimuth_correction should match Rz(90+offset)*raw for offset=" +
                    std::to_string(offsetDeg));
        }
    }

    // Sanity check in human terms: a raw reading with zero yaw/pitch/roll, once
    // ENU-corrected with zero user offset, should point East-referenced yaw=90deg
    // (i.e. North), matching the fixed convention constant used internally:
    const auto   corrected = imu_apply_enu_azimuth_correction(gtsam::Rot3::Identity(), 0.0);
    const double yaw_deg   = corrected.yaw() / kDeg2Rad;
    expect(
        std::abs(yaw_deg - 90.0) < 1e-6,
        "zero-offset correction of identity attitude should yield yaw=90deg");
}
}  // namespace

int main()
{
    try
    {
        test_imu_accel_looks_like_gravity();
        test_imu_quaternion_looks_valid();
        test_imu_apply_enu_azimuth_correction();

        std::cout << "[Success] All imu_helpers tests passed!" << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "\n[Test Failed] " << e.what() << std::endl;
        return 1;
    }
}
