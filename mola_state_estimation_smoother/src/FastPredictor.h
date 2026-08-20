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
 * @file   FastPredictor.h
 * @brief  Lock-free, sub-ms pose predictor anchored on the latest backend
 *         snapshot.
 * @author Jose Luis Blanco Claraco
 */
#pragma once

#include <mola_kernel/interfaces/NavStateFilter.h>  // NavState
#include <mola_state_estimation_smoother/Parameters.h>
#include <mrpt/core/Clock.h>

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "Snapshot.h"

namespace mola::state_estimation_smoother
{
/** Serves `estimated_navstate()` queries in sub-millisecond time, without
 *  touching the smoother's `stateMutex_` or running any factor-graph solve.
 *
 *  It holds the latest immutable Snapshot published by the backend and, on each
 *  query, extrapolates that anchor to the requested time with the configured
 *  kinematic model (identical maths to the synchronous
 *  StateEstimationSmoother::estimated_navstate(), so the two agree up to the
 *  backend's solve lag). The frame-local {odom_i} hardening is preserved: an
 *  odometry-frame query anchors on that source's own last raw pose, never on a
 *  {map}-frame reconstruction.
 *
 *  Thread-safety: its own mutex, deliberately separate from the smoother's
 *  `stateMutex_`, so a query never blocks on a backend batch solve.
 */
class FastPredictor
{
   public:
    FastPredictor() = default;

    /// Publishes a newly solved snapshot (called by the backend after a solve).
    void set_snapshot(std::shared_ptr<const Snapshot> snap);

    /// Returns the current snapshot (may be null / invalid before the first solve).
    [[nodiscard]] std::shared_ptr<const Snapshot> snapshot() const;

    /// Records the timestamp of the latest incoming observation together with
    /// the current wallclock, so a real-time "now" stamp can be computed without
    /// touching the smoother's stateMutex_.
    void note_observation_stamp(const mrpt::Clock::time_point& obsStamp);

    /// Real-time "now" stamp: the latest observation stamp advanced by the
    /// wallclock elapsed since it was received. nullopt if none seen yet.
    [[nodiscard]] std::optional<mrpt::Clock::time_point> get_current_extrapolated_stamp() const;

    /// Extrapolates the snapshot anchor to `t_query` in `frame_id`. nullopt if
    /// there is no valid snapshot, the frame is unknown, or `t_query` is beyond
    /// `max_time_to_use_velocity_model`.
    /** \param anchorStampOut If not null, receives the stamp of the snapshot
     *  anchor this very prediction was extrapolated from. Reading it from a
     *  second snapshot() call instead would race the backend thread, which may
     *  publish a newer snapshot in between. */
    [[nodiscard]] std::optional<NavState> predict(
        const mrpt::Clock::time_point& t_query, const std::string& frame_id,
        const Parameters& params, mrpt::Clock::time_point* anchorStampOut = nullptr) const;

    /// Drops the snapshot and the observation-stamp tracking (used on reset()).
    void clear();

   private:
    mutable std::mutex              mtx_;
    std::shared_ptr<const Snapshot> snapshot_;

    std::optional<mrpt::Clock::time_point> lastObsStamp_;
    mrpt::Clock::time_point                lastObsWallclock_;
};

}  // namespace mola::state_estimation_smoother
