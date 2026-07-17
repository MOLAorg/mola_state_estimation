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
 * @file   Snapshot.h
 * @brief  Immutable read-model of the smoother's latest solution.
 * @author Jose Luis Blanco Claraco
 */
#pragma once

#include <mola_kernel/Georeferencing.h>
#include <mola_kernel/interfaces/NavStateFilter.h>  // NavState
#include <mrpt/containers/bimap.h>
#include <mrpt/core/Clock.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#include <cstdint>
#include <map>
#include <optional>
#include <string>

namespace mola::state_estimation_smoother
{
/** An immutable snapshot of everything the read side of the smoother needs,
 *  produced once per backend solve and published atomically as a
 *  std::shared_ptr<const Snapshot>.
 *
 *  In synchronous mode it is built at the end of each
 *  process_pending_gtsam_updates() so the getters can read a consistent set of
 *  values; in async mode it is what the FastPredictor and the lock-free getters
 *  read while the backend thread holds stateMutex_ for the next solve.
 */
struct Snapshot
{
    using odometry_frameid_t = uint8_t;

    /// False until the first successful solve, and again after a reset(): a
    /// consumer must treat an invalid snapshot as "not ready", never as stale
    /// certainty.
    bool valid = false;

    /// Newest keyframe stamp and its optimized state (in the reference {map}
    /// frame). This is the anchor the fast predictor extrapolates from.
    mrpt::Clock::time_point anchorStamp;
    NavState                anchor;

    /// Latest estimated T_map_to_odom_i, keyed by numeric odometry frame id,
    /// plus the name<->id mapping so a query by frame name can be resolved.
    std::map<odometry_frameid_t, mrpt::poses::CPose3DPDFGaussian> frameTransforms;
    mrpt::containers::bimap<std::string, odometry_frameid_t>      frameNames;

    /// Each odometry source's own last raw pose in its own {odom_i} frame. The
    /// frame-local prediction anchors on this to stay immune to {map}
    /// corrections (see AGENTS.md / estimated_navstate()).
    struct RawSourcePose
    {
        mrpt::Clock::time_point         stamp;
        mrpt::poses::CPose3DPDFGaussian pose;  //!< in {odom_i}
    };
    std::map<odometry_frameid_t, RawSourcePose> lastRawPoseBySource;

    /// Current georeference (fixed or estimated), if any.
    std::optional<mola::Georeferencing> geoReference;

    /// Whether localization has converged (mode-aware, mirrors
    /// has_converged_localization()).
    bool localizationConverged = false;
};

}  // namespace mola::state_estimation_smoother
