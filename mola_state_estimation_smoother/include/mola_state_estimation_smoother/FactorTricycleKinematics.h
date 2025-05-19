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

#include <mola_kernel/factors/FactorBase.h>
#include <mrpt/core/exceptions.h>

namespace mola
{
/** Abstract representation of a constant-velocity tricycle kinematic motion
 * model factor between two key frames.
 *
 * \ingroup mola_kernel_factors_grp
 */
class FactorTricycleKinematics : public FactorBase
{
    DEFINE_SERIALIZABLE(FactorTricycleKinematics, mola)

   public:
    FactorTricycleKinematics() = default;

    /** Creates relative pose constraint of KF `to` as seem from `from`. */
    FactorTricycleKinematics(id_t kf_from, id_t kf_to, double deltaTime)
        : from_kf_(kf_from), to_kf_(kf_to), deltaTime_(deltaTime)
    {
    }

    id_t from_kf_ = INVALID_ID, to_kf_ = INVALID_ID;

    /** Elapsed time between "from_kf" and "to_kf" [seconds] */
    double deltaTime_ = .0;

    std::size_t edge_count() const override { return 2; }
    mola::id_t  edge_indices(const std::size_t i) const override;
};

}  // namespace mola
