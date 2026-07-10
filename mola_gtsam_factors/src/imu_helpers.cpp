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

/**
 * @file   imu_helpers.cpp
 * @brief  Shared, dependency-light helpers to validate and convert raw IMU
 *         readings into the types used by the IMU-related GTSAM factors.
 * @author Jose Luis Blanco Claraco
 * @date   Jul 10, 2026
 */

#include <mola_gtsam_factors/imu_helpers.h>

#include <cmath>

namespace
{
constexpr double kDeg2Rad = M_PI / 180.0;
}

namespace mola::factors
{

bool imu_accel_looks_like_gravity(const gtsam::Vector3& measuredAcc)
{
    const double norm = measuredAcc.norm();

    // Accept both m/s^2 (~9.8) and already-normalized (~1.0) IMU outputs:
    return std::abs(norm - 9.8) <= 2.0 || std::abs(norm - 1.0) <= 0.2;
}

bool imu_quaternion_looks_valid(double qw, double qx, double qy, double qz)
{
    if (std::isnan(qw) || std::isnan(qx) || std::isnan(qy) || std::isnan(qz))
    {
        return false;
    }

    const double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    return std::abs(norm - 1.0) <= 0.02;
}

gtsam::Rot3 imu_apply_enu_azimuth_correction(
    const gtsam::Rot3& rawAttitude, double azimuthOffsetDeg)
{
    // ENU convention: yaw=0 => East. Correct wrt true azimuth (north-referenced):
    return gtsam::Rot3::Rz((90.0 + azimuthOffsetDeg) * kDeg2Rad) * rawAttitude;
}

}  // namespace mola::factors
