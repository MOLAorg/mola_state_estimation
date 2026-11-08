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
 * @file   NewestLastFixedLagSmoother.cpp
 * @brief  GTSAM 4.2 fixed-lag smoother that keeps the newest states at the
 *         root of the Bayes tree.
 * @author Jose Luis Blanco Claraco
 */

#include "NewestLastFixedLagSmoother.h"

#if GTSAM_USES_BOOST

#include <algorithm>
#include <set>

using namespace mola::state_estimation_smoother;

namespace
{
// Copy of GTSAM 4.2's helper of the same name (not exported): the keys between
// a marginalized key and the leaves, which must be re-eliminated with it.
void recursiveMarkAffectedKeys(
    const gtsam::Key& key, const gtsam::ISAM2Clique::shared_ptr& clique,
    std::set<gtsam::Key>& additionalKeys)
{
    const auto& conditional = *clique->conditional();
    if (std::find(conditional.beginParents(), conditional.endParents(), key) ==
        conditional.endParents())
    {
        return;
    }
    for (const gtsam::Key i : conditional.frontals())
    {
        additionalKeys.insert(i);
    }
    for (const auto& child : clique->children)
    {
        recursiveMarkAffectedKeys(key, child, additionalKeys);
    }
}
}  // namespace

// Same steps as GTSAM 4.2's IncrementalFixedLagSmoother::update(), but for the
// ordering constraints.
NewestLastFixedLagSmoother::Result NewestLastFixedLagSmoother::update(
    const gtsam::NonlinearFactorGraph& newFactors, const gtsam::Values& newTheta,
    const KeyTimestampMap& timestamps, const gtsam::FactorIndices& factorsToRemove)
{
    if (!newestKeysLast_)
    {
        return gtsam::IncrementalFixedLagSmoother::update(
            newFactors, newTheta, timestamps, factorsToRemove);
    }

    updateKeyTimestampMap(timestamps);
    const double           currentTimestamp   = getCurrentTimestamp();
    const gtsam::KeyVector marginalizableKeys = findKeysBefore(currentTimestamp - smootherLag_);

    std::set<gtsam::Key> additionalKeys;
    for (const gtsam::Key key : marginalizableKeys)
    {
        for (const auto& child : isam_[key]->children)
        {
            recursiveMarkAffectedKeys(key, child, additionalKeys);
        }
    }
    const gtsam::KeyList additionalMarkedKeys(additionalKeys.begin(), additionalKeys.end());

    isamResult_ = isam_.update(
        newFactors, newTheta, factorsToRemove,
        orderingConstraints(marginalizableKeys, newFactors.keys(), currentTimestamp), boost::none,
        additionalMarkedKeys);

    if (!marginalizableKeys.empty())
    {
        const gtsam::FastList<gtsam::Key> leafKeys(
            marginalizableKeys.begin(), marginalizableKeys.end());
        isam_.marginalizeLeaves(leafKeys);
    }
    eraseKeyTimestampMap(marginalizableKeys);

    Result result;
    result.iterations = 1;
    return result;
}

boost::optional<gtsam::FastMap<gtsam::Key, int>> NewestLastFixedLagSmoother::orderingConstraints(
    const gtsam::KeyVector& marginalizableKeys, const gtsam::KeySet& newFactorKeys,
    const double currentTimestamp) const
{
    // A key without factors yet has no place in the ordering:
    const gtsam::VariableIndex& variableIndex = isam_.getVariableIndex();
    const auto                  isActive      = [&](const gtsam::Key key)
    { return newFactorKeys.exists(key) || variableIndex.find(key) != variableIndex.end(); };

    gtsam::FastMap<gtsam::Key, int> groups;
    const auto                      newestBegin = timestampKeyMap_.lower_bound(currentTimestamp);

    if (marginalizableKeys.empty())
    {
        // Only the newest keys need a group: the others stay in the default 0.
        for (auto it = newestBegin; it != timestampKeyMap_.end(); ++it)
        {
            if (isActive(it->second))
            {
                groups[it->second] = 1;
            }
        }
        for (const gtsam::Key key : newFactorKeys)
        {
            groups[key] = 1;
        }
        return groups;
    }

    // GTSAM 4.2's CCOLAMD overflows a buffer if a group number is not below the
    // number of variables ISAM2 reorders. The keys of the new factors are always
    // among those, so the newest keys get their own third group only if there
    // are at least three of them; otherwise, this update orders as GTSAM 4.2.
    const bool newestGroup = newFactorKeys.size() >= 3;
    for (const auto& [stamp, key] : timestampKeyMap_)
    {
        if (isActive(key))
        {
            groups[key] = newestGroup && stamp >= currentTimestamp ? 2 : 1;
        }
    }
    if (newestGroup)
    {
        for (const gtsam::Key key : newFactorKeys)
        {
            groups[key] = 2;
        }
    }
    for (const gtsam::Key key : marginalizableKeys)
    {
        groups[key] = 0;
    }
    return groups;
}

#endif
