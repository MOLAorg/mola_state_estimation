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
 * @file   imu_preintegration.h
 * @brief  Integration of buffered raw IMU readings into a GTSAM preintegrated
 *         measurement between two instants.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 12, 2026
 */

#pragma once

#include <gtsam/navigation/AHRSFactor.h>
#include <gtsam/navigation/ImuFactor.h>

#include <deque>

namespace mola::factors
{
/** One raw IMU reading, in the sensor frame: specific force (before any unit
 *  scaling) and angular velocity, sampled at time `t` [s]. */
struct ImuReading
{
    double         t = 0;
    gtsam::Vector3 acc;
    gtsam::Vector3 gyr;
};

/** Integrates `readings` (oldest first, strictly increasing stamps) over
 *  [ti, tj] into `pim`, whose parameters (gravity, body_P_sensor) and bias
 *  linearization point are kept. Accelerations are multiplied by `accScale`.
 *
 *  Readings are taken as instantaneous samples, linearly interpolated in
 *  between. With an IMU away from the body origin (body_P_sensor), the
 *  tangential acceleration of the lever arm, which gtsam does not model, is
 *  compensated from the change of the gyro readings.
 *
 *  Integration starts at the last reading at or before `ti` (or at the first
 *  one, if it comes at most `maxReadingGap` after `ti`), and fails if two
 *  consecutive readings are farther apart than `maxReadingGap` or the
 *  readings do not reach `tj`.
 *
 *  \return true if [ti, tj] was fully integrated.
 */
bool integrate_imu_readings(
    const std::deque<ImuReading>& readings, double ti, double tj, double accScale,
    double maxReadingGap, gtsam::PreintegratedImuMeasurements& pim);

/** As integrate_imu_readings(), for the gyroscope only (rotation preintegration):
 *  same reading interpolation and failure conditions; the accelerometer is
 *  ignored. Readings are rotated into the vehicle frame (`R_body_sensor`) before
 *  integration, so `pim` must have no body_P_sensor (see FactorGyroPreintegration). */
bool integrate_gyro_readings(
    const std::deque<ImuReading>& readings, double ti, double tj, double maxReadingGap,
    const gtsam::Rot3& R_body_sensor, gtsam::PreintegratedAhrsMeasurements& pim);

}  // namespace mola::factors
