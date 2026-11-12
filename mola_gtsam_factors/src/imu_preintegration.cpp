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

    // Zero-order hold: each reading holds until the next one. Start at the
    // last reading at or before ti (or the first one, if close enough).
    auto first = std::upper_bound(
        readings.begin(), readings.end(), ti,
        [](double v, const ImuReading& r) { return v < r.t; });
    if (first == readings.begin())
    {
        if (first->t - ti > maxReadingGap)
        {
            return false;
        }
    }
    else
    {
        --first;
    }

    const double t0 = pim.deltaTij();
    double       tc = ti;
    for (auto k = first; k != readings.end() && tc < tj; ++k)
    {
        const auto   next  = std::next(k);
        const double tNext = (next != readings.end()) ? std::min(next->t, tj) : tj;
        if (next != readings.end() && next->t - k->t > maxReadingGap)
        {
            return false;
        }
        if (tNext > tc)
        {
            pim.integrateMeasurement(accScale * k->acc, k->gyr, tNext - tc);
        }
        tc = tNext;
    }
    return std::abs(pim.deltaTij() - t0 - (tj - ti)) < 1e-6;
}

}  // namespace mola::factors
