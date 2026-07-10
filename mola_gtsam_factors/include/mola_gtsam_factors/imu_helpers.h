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
 * @file   imu_helpers.h
 * @brief  Shared, dependency-light helpers to validate and convert raw IMU
 *         readings into the types used by the IMU-related GTSAM factors.
 * @author Jose Luis Blanco Claraco
 * @date   Jul 10, 2026
 */

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Rot3.h>

namespace mola::factors
{
/** Returns true if the given accelerometer reading looks like a plausible
 *  gravity vector, accepting either raw m/s^2 units (norm ~9.8) or an
 *  already-normalized ("g") unit vector (norm ~1.0), as reported by
 *  different IMU drivers.
 */
bool imu_accel_looks_like_gravity(const gtsam::Vector3& measuredAcc);

/** Returns true if the given IMU absolute-orientation quaternion (w,x,y,z)
 *  is finite and normalized, i.e. safe to convert into a gtsam::Rot3.
 */
bool imu_quaternion_looks_valid(double qw, double qx, double qy, double qz);

/** Applies the fixed ENU-convention correction (yaw=0 => East) plus a
 *  user-calibrated azimuth offset (e.g. sensor mounting or magnetic
 *  declination) to a raw IMU attitude reading, so the result can be
 *  compared against poses expressed in the ENU frame.
 */
gtsam::Rot3 imu_apply_enu_azimuth_correction(
    const gtsam::Rot3& rawAttitude, double azimuthOffsetDeg);

}  // namespace mola::factors
