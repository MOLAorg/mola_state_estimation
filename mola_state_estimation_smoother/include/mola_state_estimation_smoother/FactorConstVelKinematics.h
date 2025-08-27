/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

/**
 * @file   FactorConstVelKinematics.h
 * @brief  Constant-velocity model factor
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2019
 */
#pragma once

#include <mola_state_estimation_smoother/id.h>
#include <mrpt/core/exceptions.h>

namespace mola
{
/** Abstract representation of a constant-velocity kinematic motion model factor
 * between two key frames.
 */
class FactorConstVelKinematics
{
   public:
    FactorConstVelKinematics() = default;

    /** Creates relative pose constraint of KF `to` as seem from `from`. */
    FactorConstVelKinematics(
        id_t kf_from, id_t kf_to, double delta_time)  // NOLINT
        : from_kf(kf_from), to_kf(kf_to), deltaTime(delta_time)
    {
    }

    id_t from_kf = INVALID_ID, to_kf = INVALID_ID;

    /** Elapsed time between "from_kf" and "to_kf" [seconds] */
    double deltaTime = .0;
};

}  // namespace mola
