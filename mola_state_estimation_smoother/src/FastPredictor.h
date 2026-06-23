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
 * @brief  High-rate pose predictor anchored on the latest backend solution.
 * @author Jose Luis Blanco Claraco
 * @date   Jun 22, 2026
 */
#pragma once

#include <mola_kernel/interfaces/NavStateFilter.h>  // NavState
#include <mola_state_estimation_smoother/Parameters.h>
#include <mrpt/core/Clock.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/system/datetime.h>  // timeDifference

#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <string>

namespace mola::state_estimation_smoother
{
/** Lightweight, high-rate (20-50 Hz) pose predictor used when the smoother runs
 *  in `async_backend` mode.
 *
 *  It keeps the most recent optimized state ("anchor") produced by the backend
 *  thread, plus a short buffer of high-rate observations that arrived after it.
 *  On each `predict()` query it builds a *tiny* GTSAM factor graph re-anchored at
 *  that solution (a Gaussian prior at the anchor + constant-velocity kinematic
 *  factors + the buffered high-rate sensors), solves it, and returns the
 *  extrapolated vehicle state in the reference frame. Each solve touches only a
 *  handful of variables, so it stays sub-millisecond and never blocks on the
 *  long backend batch solve.
 *
 *  Thread-safety: all public methods are guarded by an internal mutex that is
 *  deliberately separate from the smoother's `stateMutex_`, so fast queries are
 *  not blocked while the backend holds `stateMutex_` for a full batch update.
 */
class FastPredictor
{
   public:
    FastPredictor() = default;

    /// Latest estimated T_map_to_frame transforms, keyed by odometry frame name.
    using FrameTransforms = std::map<std::string, mrpt::poses::CPose3DPDFGaussian>;

    /** Replaces the anchor with a newly optimized state (called by the backend
     *  thread after each successful solve). `anchorState` is expressed in the
     *  reference frame; `frameTransforms` carries the latest T_map_to_frame for
     *  each known odometry frame_id, so the fast query path can convert results
     *  to a requested frame without touching the smoother's `stateMutex_`.
     *  Buffered observations older than the new anchor are dropped.
     */
    void set_anchor(
        const NavState& anchorState, const mrpt::Clock::time_point& anchorStamp,
        const FrameTransforms& frameTransforms);

    /** Returns the cached T_map_to_frame for `frame_id`, or nullopt if unknown. */
    [[nodiscard]] std::optional<mrpt::poses::CPose3DPDFGaussian> frame_transform(
        const std::string& frame_id) const;

    /** Buffers a high-rate observation to be re-fused on top of the anchor.
     *  Only the observation classes the fast layer uses (wheel odometry) are
     *  retained; others are ignored. The buffer is trimmed to
     *  `bufferLength` seconds of history.
     */
    void push_observation(const mrpt::obs::CObservation::ConstPtr& o, double bufferLength);

    /** Records the timestamp of the latest incoming observation (of any class)
     *  together with the current wallclock, so spinOnce() can compute a timely
     *  "now" stamp without touching the smoother's stateMutex_. */
    void note_observation_stamp(const mrpt::Clock::time_point& obsStamp);

    /** Real-time "now" stamp: the latest observation stamp advanced by the
     *  wallclock elapsed since it was received. std::nullopt if no observation
     *  has been seen yet. Lock-free w.r.t. the backend solve (own mutex only). */
    [[nodiscard]] std::optional<mrpt::Clock::time_point> get_current_extrapolated_stamp() const;

    /** Builds and solves the tiny re-anchored graph, returning the estimated
     *  vehicle state at `t_query` in the reference frame. Returns std::nullopt
     *  if there is no anchor yet, if `t_query` is too far beyond the latest data
     *  (`max_time_to_use_velocity_model`), or if the tiny solve is degenerate.
     */
    [[nodiscard]] std::optional<NavState> predict(
        const mrpt::Clock::time_point& t_query, const Parameters& params) const;

    /** Drops the anchor and all buffered observations (used on reset()). */
    void clear();

    /** Returns true if an anchor has been set. */
    [[nodiscard]] bool has_anchor() const;

   private:
    mutable std::mutex mtx_;

    std::optional<NavState>                       anchor_;
    mrpt::Clock::time_point                       anchorStamp_;
    FrameTransforms                               frameTransforms_;
    std::deque<mrpt::obs::CObservation::ConstPtr> buffer_;  //!< wheel-odom history, time-sorted

    std::optional<mrpt::Clock::time_point> lastObsStamp_;
    mrpt::Clock::time_point                lastObsWallclock_;
};

}  // namespace mola::state_estimation_smoother
