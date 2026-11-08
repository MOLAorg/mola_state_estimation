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
 * @file   NewestLastFixedLagSmoother.h
 * @brief  GTSAM 4.2 fixed-lag smoother that keeps the newest states at the
 *         root of the Bayes tree.
 * @author Jose Luis Blanco Claraco
 */
#pragma once

#include <mola_gtsam_factors/gtsam_detect_version.h>

#if __has_include(<gtsam/nonlinear/IncrementalFixedLagSmoother.h>)
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#else
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#endif

namespace mola::state_estimation_smoother
{
#if GTSAM_USES_BOOST
/** GTSAM 4.2's IncrementalFixedLagSmoother, optionally eliminating the newest
 *  keys last: those of the new factors and those at the current timestamp.
 *  The base class only constrains the marginalizable keys to go first, which
 *  can leave the newest states deep in the Bayes tree, so that each update
 *  re-eliminates, and each marginal of the current state crosses, a long path
 *  of it. It also re-eliminates the cliques of the marginalized keys, which
 *  4.2's marginalization needs first among the frontals of their cliques.
 */
class NewestLastFixedLagSmoother : public gtsam::IncrementalFixedLagSmoother
{
   public:
    NewestLastFixedLagSmoother(
        double smootherLag, const gtsam::ISAM2Params& parameters, bool newestKeysLast)
        : gtsam::IncrementalFixedLagSmoother(smootherLag, parameters),
          newestKeysLast_(newestKeysLast)
    {
    }

    /** As in the base class, with the ordering constraints described above. */
    Result update(
        const gtsam::NonlinearFactorGraph& newFactors      = gtsam::NonlinearFactorGraph(),
        const gtsam::Values&               newTheta        = gtsam::Values(),
        const KeyTimestampMap&             timestamps      = KeyTimestampMap(),
        const gtsam::FactorIndices&        factorsToRemove = gtsam::FactorIndices()) override;

   protected:
    /** The constraint groups of an update with these marginalizable keys and
     * new-factor keys, at this current timestamp. */
    boost::optional<gtsam::FastMap<gtsam::Key, int>> orderingConstraints(
        const gtsam::KeyVector& marginalizableKeys, const gtsam::KeySet& newFactorKeys,
        double currentTimestamp) const;

   private:
    bool newestKeysLast_;
};
#else
// Other GTSAM versions keep their own ordering.
using NewestLastFixedLagSmoother = gtsam::IncrementalFixedLagSmoother;
#endif

}  // namespace mola::state_estimation_smoother
