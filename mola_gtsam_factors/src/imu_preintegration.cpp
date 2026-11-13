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
 * @file   imu_preintegration.cpp
 * @brief  Integration of buffered raw IMU readings into a GTSAM preintegrated
 *         measurement between two instants.
 * @author Jose Luis Blanco Claraco
 * @date   Nov 12, 2026
 */

#include <mola_gtsam_factors/imu_preintegration.h>

#include <algorithm>
#include <cmath>
#include <iterator>

namespace mola::factors
{
bool integrate_imu_readings(
    const std::deque<ImuReading>& readings, double ti, double tj, double accScale,
    double maxReadingGap, gtsam::PreintegratedImuMeasurements& pim)
{
    if (readings.empty() || tj <= ti)
    {
        return false;
    }

    // Start at the last reading at or before ti (or the first one, if close
    // enough, held back to ti).
    auto k = std::upper_bound(
        readings.begin(), readings.end(), ti,
        [](double v, const ImuReading& r) { return v < r.t; });
    if (k == readings.begin())
    {
        if (k->t - ti > maxReadingGap)
        {
            return false;
        }
    }
    else
    {
        --k;
    }

    // gtsam corrects the accelerometer of an IMU away from the body origin by
    // the centripetal term only, w x (w x r). The tangential one, dw/dt x r,
    // is left out, though it reaches several m/s^2 with a lever arm of tens of
    // cm under brisk rotations. It is removed here, with dw/dt from consecutive
    // gyro readings: over a whole interval its integral only depends on the
    // gyro at both ends, so differentiating the readings does not amplify
    // their noise in the integrated result. The gyro bias cancels out.
    const auto&          bPs   = pim.params()->body_P_sensor;
    const bool           lever = bPs.has_value() && bPs->translation().norm() > 0;
    const gtsam::Rot3    R_bs  = lever ? bPs->rotation() : gtsam::Rot3();
    const gtsam::Vector3 leverB =
        lever ? gtsam::Vector3(bPs->translation()) : gtsam::Vector3::Zero();

    // Readings are samples of continuous signals: each sub-interval between
    // consecutive readings (or interval ends) is integrated with the signals
    // linearly interpolated at its midpoint, which is second-order accurate
    // where holding each reading until the next lags by half a period.
    const double t0 = pim.deltaTij();
    double       tc = ti;

    // Before the first reading, its values are held: there is nothing to
    // interpolate with, nor an angular acceleration to apply.
    if (tc < k->t)
    {
        const double tEnd = std::min(k->t, tj);
        pim.integrateMeasurement(accScale * k->acc, k->gyr, tEnd - tc);
        tc = tEnd;
    }

    while (tc < tj)
    {
        const auto next = std::next(k);
        if (next == readings.end())
        {
            return false;  // the readings do not reach tj
        }
        const double span = next->t - k->t;
        if (span > maxReadingGap)
        {
            return false;
        }
        const double tEnd = std::min(next->t, tj);
        if (tEnd > tc)
        {
            const double         s   = std::clamp((0.5 * (tc + tEnd) - k->t) / span, 0.0, 1.0);
            gtsam::Vector3       acc = accScale * ((1 - s) * k->acc + s * next->acc);
            const gtsam::Vector3 gyr = (1 - s) * k->gyr + s * next->gyr;
            if (lever)
            {
                const gtsam::Vector3 angAccB = R_bs.rotate((next->gyr - k->gyr) / span);
                acc -= R_bs.unrotate(angAccB.cross(leverB));
            }
            pim.integrateMeasurement(acc, gyr, tEnd - tc);
        }
        tc = tEnd;
        k  = next;
    }
    return std::abs(pim.deltaTij() - t0 - (tj - ti)) < 1e-6;
}

}  // namespace mola::factors
