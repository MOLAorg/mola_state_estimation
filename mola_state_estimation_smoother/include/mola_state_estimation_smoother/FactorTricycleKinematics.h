/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   FactorTricycleKinematics.h
 * @brief  Constant-velocity model factor for tricycle motion model
 * @author Jose Luis Blanco Claraco
 * @date   Jan 18, 2025
 */
#pragma once

#include <mola_state_estimation_smoother/id.h>
#include <mrpt/core/exceptions.h>

namespace mola
{
/** Abstract representation of a constant-velocity tricycle kinematic motion
 * model factor between two key frames.
 */
class FactorTricycleKinematics
{
   public:
    FactorTricycleKinematics() = default;

    /** Creates relative pose constraint of KF `to` as seem from `from`. */
    FactorTricycleKinematics(
        id_t kf_from, id_t kf_to, double delta_time)  // NOLINT
        : from_kf(kf_from), to_kf(kf_to), deltaTime(delta_time)
    {
    }

    id_t from_kf = INVALID_ID, to_kf = INVALID_ID;

    /** Elapsed time between "from_kf" and "to_kf" [seconds] */
    double deltaTime = .0;
};

}  // namespace mola
